"""Peer state machine; transport and clock are injected."""
import copy
from .messages import Message
from .task_ledger import Ledger
from .bidder import make_bid
from .cost import bid_key, edge_key
from .fault_detector import local_reason, stale
from .predictors import make_predictor

class Node:
    def __init__(self, robot_id, config, send, clock, log=None, state_path=None):
        self.id, self.config, self.send, self.clock = robot_id, config, send, clock
        self.log = log or (lambda event: None)
        self.robot = copy.deepcopy(config['robots'][robot_id])
        self.ledger = Ledger(config['members'], config['clock_skew_bound'], state_path)
        self.tasks = {t['task_id']: copy.deepcopy(t) for t in config['tasks']}
        self.auctions, self.heartbeats, self.blocks = {}, {self.id: self.now}, {}
        self.seen, self.outbox, self.sequence = {}, {}, 0
        self.reservation, self.pending = None, None
        self.offers = {}
        self.predictor = make_predictor(config)
        self.peer_info = {}
        self.auctions_enabled = True
        self.enabled, self.auto_detect = True, True
        self.last_heartbeat = -1e30
        self.routes, self.reassignments, self.retry_due = {}, {}, {}
        self.started_at = self.now
        self.event('config', weights=config['weights'])

    @property
    def now(self):
        return self.clock()

    @property
    def blocked(self):
        return {key for key, value in self.blocks.items() if value['blocked']}

    def event(self, kind, **fields):
        self.log(dict(t=round(self.now, 6), robot=self.id, kind=kind, **fields))

    def publish(self, kind, payload, repeat=0):
        self.sequence += 1
        m = Message.make(kind, self.id, self.now, payload,
                         f'{self.id}:{self.started_at}:{self.sequence}')
        self.send(m)
        if repeat:
            self.outbox[m.message_id] = [m, self.now + .1, repeat]
        self.event('send_' + kind, **payload)
        return m

    def start(self, task_id, reason='blocked_path', attempt=0):
        if not self.auctions_enabled:
            return None
        if not self.enabled or self.tasks[task_id].get('carrying_cargo'):
            self.event('rescue_required', task_id=task_id)
            return None
        old = self.auctions.get(task_id)
        if old and old['state'] not in {'DONE', 'CANCELLED', 'FAILED'}:
            return old['key']
        if self.ledger.entries.get(task_id, {}).get('status') == 'done':
            return None
        if self.reassignments.get(task_id, 0) >= self.config['reassignment_limit']:
            self.event('human_required', task_id=task_id)
            return None
        epoch = self.ledger.next_epoch(task_id)
        key = [epoch, self.id]
        payload = dict(task_id=task_id, epoch=epoch, initiator_id=self.id, key=key,
                       task=self.tasks[task_id], reason=reason, origin_node=self.robot['node'],
                       radius=self.config['announcement_radius'] + attempt * self.config['radius_step'],
                       attempt=attempt, bid_window_ms=self.config['bid_window'] * 1000,
                       closes=self.now + self.config['bid_window'])
        self.publish('announce', payload, 3)
        return key

    def identity(self, auction):
        return {k: auction[k] for k in ('task_id', 'epoch', 'key')}

    def receive(self, m):
        if not self.enabled or m.sender not in self.config['members']:
            return
        if m.message_id in self.seen:
            return
        self.seen[m.message_id] = self.now
        p, kind = m.payload, m.kind
        if kind == 'heartbeat':
            # Heartbeats are freshness hints, not ownership authority.
            if abs(self.now - m.timestamp) <= self.config['heartbeat_timeout']:
                self.heartbeats[m.sender] = self.now
                self.peer_info[m.sender] = dict(
                    node=p.get('node'), speed=p.get('speed', 0.0), route=p.get('route', []),
                    battery=p.get('battery'), priority=p.get('priority', 0), seen=self.now)
                for tid, entry in p.get('entries', {}).items():
                    if tid not in self.tasks:
                        continue
                    proposal = {k: v for k, v in entry.items() if k not in {'status', 'voters'}}
                    self.apply_commit(proposal, entry['voters'])
                    if entry['status'] == 'done':
                        self.ledger.merge_done(entry)
                        if tid in self.auctions:
                            self.auctions[tid]['state'] = 'DONE'
                for key, block in p.get('blocks', {}).items():
                    previous = self.blocks.get(key)
                    if previous is None or tuple(block['version']) > tuple(previous['version']):
                        self.blocks[key] = block
            return
        if kind == 'blocked_edges':
            previous = self.blocks.get(p['edge'])
            if previous is None or tuple(p['version']) > tuple(previous['version']):
                self.blocks[p['edge']] = p
            return
        tid = p.get('task_id')
        if tid not in self.tasks:
            return
        if kind in {'lease_request', 'lease_grant', 'commit', 'release'}:
            self.control(m)
            return
        if kind == 'announce':
            if p['initiator_id'] != m.sender or p['key'] != [p['epoch'], m.sender]:
                return
            old = self.auctions.get(tid)
            if old and tuple(old['key']) >= tuple(p['key']):
                return
            if p['closes'] < self.now or p['task'] != self.tasks[tid]:
                return
            if p['closes'] > self.now + self.config['bid_window'] + self.config['clock_skew_bound']:
                return
            self.ledger.observe_epoch(tid, p['epoch'])
            self.auctions[tid] = a = dict(p, state='COLLECTING_BIDS', bids={}, rejected=[],
                                         winner=None, deadline=0, award_sender=None)
            bid, reason = make_bid(self, a)
            self.publish('bid' if bid else 'decline',
                         dict(self.identity(a), **(bid or {'reason': reason})), 2)
            return
        a = self.auctions.get(tid)
        if not a or p.get('key') != a['key']:
            return
        if kind == 'bid':
            if self.now > a['closes'] or a['state'] != 'COLLECTING_BIDS':
                return
            if stale(self.heartbeats.get(m.sender, -1e30), self.now, self.config['heartbeat_timeout']):
                return
            if p.get('bidder_id') == m.sender:
                a['bids'][m.sender] = p
                self.event('bid_received', task_id=tid, bid=p)
        elif kind == 'award':
            if a['state'] in {'DONE', 'CANCELLED', 'FAILED', 'CONFIRMED'} or m.sender != self.coordinator(a):
                return
            if p.get('winner_id') not in a['bids']:
                return
            a.update(state='AWARDED', winner=p['winner_id'], deadline=p['confirm_by'], award_sender=m.sender)
            if p['winner_id'] == self.id:
                self.offers.setdefault(tid, (self.now + .05, a))
        elif kind == 'confirm':
            if m.sender != a['winner'] or a['state'] != 'AWARDED':
                return
            if p['accepted']:
                self.event('confirmation_received', task_id=tid, winner=m.sender)
            else:
                a['rejected'].append(m.sender)
                a['state'] = 'COLLECTING_BIDS'
        elif kind == 'cancel':
            if m.sender == a['initiator_id'] and a['state'] not in {'DONE', 'CONFIRMED'}:
                a['state'] = 'CANCELLED'
                self.event('cancelled', task_id=tid, reason=p['reason'])
                if self.pending and self.pending['proposal']['task_id'] == tid:
                    self.pending = None
                    self.reservation = None

    def coordinator(self, a):
        alive = sorted(r for r in self.config['members']
                       if not stale(self.heartbeats.get(r, -1e30), self.now,
                                    self.config['heartbeat_timeout']))
        return a['initiator_id'] if a['initiator_id'] in alive else (alive[0] if alive else None)

    def award(self, a):
        eligible = [b for rid, b in a['bids'].items() if rid not in a['rejected']
                    and not stale(self.heartbeats.get(rid, -1e30), self.now,
                                  self.config['heartbeat_timeout'])]
        if not eligible:
            a['state'] = 'FAILED'
            if a['attempt'] < self.config['retry_count']:
                self.retry_due[a['task_id']] = (self.now + self.config['retry_backoff'] * (a['attempt'] + 1),
                                                 a['attempt'] + 1)
            else:
                self.event('unassigned', task_id=a['task_id'])
            return
        winner = min(eligible, key=bid_key)
        a.update(state='AWARDED', winner=winner['bidder_id'],
                 deadline=self.now + self.config['confirm_timeout'])
        self.publish('award', dict(self.identity(a), winner_id=a['winner'],
                                  winning_cost=winner['total_cost'], confirm_by=a['deadline']), 2)

    def accept_award(self, a):
        if self.pending and self.pending['proposal']['key'] == a['key'] and self.pending['proposal']['task_id'] == a['task_id']:
            return
        bid, reason = make_bid(self, a)
        if bid is None or self.now > a['deadline']:
            self.publish('confirm', dict(self.identity(a), winner_id=self.id,
                                         accepted=False, new_route_id=None, reason=reason or 'expired'), 2)
            return
        tid = a['task_id']
        route_id = f"{tid}:{a['epoch']}:{a['key'][1]}:{self.id}"
        # Reserve atomically in this single-threaded event loop.
        self.reservation = (tid, self.now + self.config['lease_seconds'])
        proposal = dict(self.identity(a), owner=self.id, token=route_id,
                        lease_end=self.now + self.config['lease_seconds'],
                        route=bid['route'], route_id=route_id,
                        assignment_count=self.reassignments.get(tid, 0) + 1)
        self.pending = dict(proposal=proposal, voters=set(),
                            deadline=self.now + self.config['proposal_timeout'])
        self.publish('confirm', dict(self.identity(a), winner_id=self.id,
                                     accepted=True, new_route_id=route_id), 2)
        self.publish('lease_request', proposal, 4)

    def control(self, m):
        p, tid = m.payload, m.payload['task_id']
        if m.kind == 'lease_request':
            if m.sender != p['owner'] or p['lease_end'] > self.now + self.config['lease_seconds'] + self.config['clock_skew_bound']:
                return
            a = self.auctions.get(tid)
            if not a or a['key'] != p['key'] or a['state'] in {'CANCELLED', 'DONE', 'FAILED'}:
                return
            if self.ledger.grant(p, self.now):
                self.publish('lease_grant', dict(task_id=tid, proposal=p, voter=self.id), 3)
        elif m.kind == 'lease_grant':
            if not self.pending or p['voter'] != m.sender or p['proposal'] != self.pending['proposal']:
                return
            self.pending['voters'].add(m.sender)
            if len(self.pending['voters']) >= self.ledger.majority:
                proposal = self.pending['proposal']
                voters = sorted(self.pending['voters'])
                self.publish('commit', dict(task_id=tid, proposal=proposal, voters=voters), 6)
                self.apply_commit(proposal, voters)
                self.pending = None
        elif m.kind == 'commit':
            if m.sender == p['proposal']['owner']:
                self.apply_commit(p['proposal'], p['voters'])
        elif m.kind == 'release':
            entry = self.ledger.entries.get(tid, {})
            if m.sender == entry.get('owner') and p['token'] == entry.get('token'):
                self.ledger.done(tid, p['token'])
                if tid in self.auctions:
                    self.auctions[tid]['state'] = 'DONE'
                self.event('task_done', task_id=tid)

    def apply_commit(self, proposal, voters):
        tid = proposal['task_id']
        if self.ledger.commit(proposal, voters, self.now):
            self.reassignments[tid] = max(self.reassignments.get(tid, 0), proposal['assignment_count'])
            a = self.auctions.get(tid)
            if a and a['key'] == proposal['key']:
                a['state'] = 'CONFIRMED'
            self.routes[tid] = proposal['route']
            self.event('ownership_committed', task_id=tid, owner=proposal['owner'],
                       token=proposal['token'])

    def complete(self, tid):
        if self.may_execute(tid):
            token = self.ledger.entries[tid]['token']
            self.ledger.done(tid, token)
            self.publish('release', dict(task_id=tid, token=token), 6)
            if tid in self.auctions:
                self.auctions[tid]['state'] = 'DONE'
            self.reservation = None
            self.pending = None
            self.event('completed_locally', task_id=tid)
            return True
        return False

    def cancel(self, tid, reason='blockage_cleared'):
        a = self.auctions.get(tid)
        if a and a['initiator_id'] == self.id:
            self.publish('cancel', dict(self.identity(a), reason=reason), 3)

    def set_block(self, start, end, blocked=True):
        key = edge_key(start, end)
        version = [max(self.blocks.get(key, {}).get('version', [0])[0] + 1,
                       int(self.now * 1000000)), self.id]
        self.publish('blocked_edges', dict(edge=key, blocked=blocked, version=version), 4)

    def tick(self):
        if not self.enabled:
            return
        now = self.now
        if now - self.last_heartbeat >= self.config['heartbeat_interval']:
            self.heartbeats[self.id] = now
            self.publish('heartbeat', dict(
                node=self.robot['node'], speed=self.robot.get('speed', 0.0),
                route=self.robot.get('intent_route', []), battery=self.robot['battery'],
                priority=self.robot.get('priority', 0),
                entries=self.ledger.entries, blocks=self.blocks))
            self.last_heartbeat = now
        for mid, item in list(self.outbox.items()):
            if now >= item[1]:
                self.send(item[0])
                item[2] -= 1
                item[1] = now + .1 * (2 ** max(0, 3 - item[2]))
                if item[2] <= 0:
                    del self.outbox[mid]
        self.seen = {mid: when for mid, when in self.seen.items() if now - when < 60}
        if self.reservation and now >= self.reservation[1]:
            self.reservation = None
        if self.pending and now > self.pending['deadline']:
            self.event('quorum_timeout', task_id=self.pending['proposal']['task_id'])
            self.pending = None
        ready = ([a for due, a in self.offers.values()]
                 if self.offers and now >= min(due for due, a in self.offers.values()) else [])
        ready.sort(key=lambda a: (-a['task']['priority'], a['task']['deadline'],
                                 a['bids'].get(self.id, {}).get('total_cost', 10**30), a['task_id']))
        for a in ready:
            self.offers.pop(a['task_id'], None)
            if a['state'] == 'AWARDED':
                self.accept_award(a)
        self.renew()
        for tid, (due, attempt) in list(self.retry_due.items()):
            if now >= due:
                del self.retry_due[tid]
                self.start(tid, 'retry', attempt)
        for tid, a in list(self.auctions.items()):
            if self.coordinator(a) == self.id:
                if a['state'] == 'COLLECTING_BIDS' and now >= a['closes'] + self.config['clock_skew_bound']:
                    self.award(a)
                elif a['state'] == 'AWARDED' and now > a['deadline'] + self.config['proposal_timeout']:
                    a['rejected'].append(a['winner'])
                    a['state'] = 'COLLECTING_BIDS'
            entry = self.ledger.entries.get(tid, {})
            if a['state'] == 'CONFIRMED' and now > entry.get('lease_end', 0) + self.config['clock_skew_bound']:
                a['state'] = 'FAILED'
                self.event('lease_expired', task_id=tid)
        if self.auto_detect:
            self.detect()

    def detect(self):
        alive = sorted(r for r in self.config['members'] if not stale(
            self.heartbeats.get(r, -1e30), self.now, self.config['heartbeat_timeout']))
        for tid, entry in list(self.ledger.entries.items()):
            if entry['status'] == 'done':
                continue
            owner = entry['owner']
            reason = local_reason(self.robot, self.tasks[tid], self.config, self.now) if owner == self.id else None
            if reason:
                # Stop locally first. Votes remain locked until their original expiry.
                self.robot['fault'] = reason
                a = self.auctions.get(tid)
                if a and a['state'] == 'CONFIRMED':
                    a['state'] = 'FAILED'
                self.start(tid, reason)
            elif alive and alive[0] == self.id and (
                stale(self.heartbeats.get(owner, -1e30), self.now, self.config['heartbeat_timeout'])
                or self.now > entry['lease_end'] + self.config['clock_skew_bound']):
                self.start(tid, 'owner_lost_or_lease_expired')

    def snapshot(self):
        return dict(robot=self.id, now=self.now, auctions=list(self.auctions.values()),
                    ledger=self.ledger.entries, blocked_edges=sorted(self.blocked), routes=self.routes)

    def may_execute(self, tid):
        return (self.enabled and not local_reason(self.robot, self.tasks[tid], self.config, self.now)
                and self.ledger.can_execute(tid, self.id, self.now))

    def renew(self):
        if self.pending:
            return
        for tid, entry in self.ledger.entries.items():
            if self.may_execute(tid) and entry['lease_end'] - self.now < self.config['lease_seconds'] / 2:
                proposal = {k: v for k, v in entry.items() if k not in {'status', 'voters'}}
                proposal['lease_end'] = self.now + self.config['lease_seconds']
                self.pending = dict(proposal=proposal, voters=set(),
                                    deadline=self.now + self.config['proposal_timeout'])
                self.publish('lease_request', proposal, 4)
                break

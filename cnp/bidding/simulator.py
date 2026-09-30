"""Seeded discrete-time network; no hidden dispatcher in the protocol."""
import copy
import heapq
import json
import random
from .initiator import Node
from .cost import evaluate

class Simulation:
    def __init__(self, config, seed=None, enabled=True, mode=None):
        # mode: 'cnp' (auctions), 'keep_owner' (no reassignment baseline), 'off' (nothing runs)
        self.mode = mode or ('cnp' if enabled else 'off')
        self.config = copy.deepcopy(config)
        self.rng = random.Random(config['seed'] if seed is None else seed)
        self.time, self.serial = 0.0, 0
        self.queue, self.events = [], []
        self.drop, self.delay, self.duplicate = 0.0, .015, 0.0
        self.partitions, self.killed, self.suppress = [], set(), set()
        self.messages = 0
        self.auction_transmissions = {}
        self.execution_ticks, self.completed = {}, set()
        self.completed_at, self.task_start = {}, 0.0
        self.duplicate_assignments = 0
        self.nodes = {rid: Node(rid, self.config, self.send, lambda: self.time, self.events.append)
                      for rid in config['members']}
        for node in self.nodes.values():
            node.enabled = self.mode != 'off'
            if self.mode == 'keep_owner':
                # baseline: robots still talk (heartbeats, blocked edges) but never auction
                node.auctions_enabled = False
                node.auto_detect = False

    def task_seconds(self, route):
        """Abstract executor: route length / execution speed (or 1 s if the config has no speed)."""
        speed = self.config.get('execution_speed')
        if not speed or not route:
            return 1.0
        graph = self.config['graph']
        length = sum(graph[a][b] for a, b in zip(route, route[1:]))
        return max(1.0, length / speed)

    def baseline_execute(self, duration):
        """No-reassignment baseline: the original owner keeps the task and detours on its own."""
        for tid, task in self.nodes[sorted(self.nodes)[0]].tasks.items():
            if tid in self.completed:
                continue
            owner_id = task.get('initial_owner', sorted(self.nodes)[0])
            node = self.nodes[owner_id]
            if owner_id in self.killed:
                continue
            bid, _ = evaluate(node.robot, dict(task, carrying_cargo=False), self.config['graph'],
                              node.blocked, self.config, self.time, node.predictor,
                              node.peer_info, owner_id)
            if bid is None:
                continue  # cannot do it: fault, no battery, no path, in safety fallback ...
            key = (tid, owner_id)
            self.execution_ticks[key] = self.execution_ticks.get(key, 0) + duration
            if self.execution_ticks[key] >= self.task_seconds(bid['route']):
                self.completed.add(tid)
                self.completed_at[tid] = self.time

    def send(self, message):
        self.messages += 1
        p = message.payload
        if 'task_id' in p:
            key = p.get('key', p.get('proposal', {}).get('key', ['control']))
            label = p['task_id'] + ':' + ':'.join(map(str, key))
            self.auction_transmissions[label] = self.auction_transmissions.get(label, 0) + 1
        for recipient in sorted(self.nodes):
            if message.sender in self.killed or recipient in self.killed:
                continue
            if message.kind in self.suppress or self.rng.random() < self.drop:
                continue
            if self.partitions and not any(message.sender in group and recipient in group
                                           for group in self.partitions):
                continue
            for _ in range(1 + int(self.rng.random() < self.duplicate)):
                self.serial += 1
                heapq.heappush(self.queue, (self.time + self.rng.uniform(0, self.delay),
                                            self.serial, recipient, copy.deepcopy(message)))

    def step(self, duration=.02, execute=False):
        self.time = round(self.time + duration, 8)
        while self.queue and self.queue[0][0] <= self.time:
            _, _, rid, message = heapq.heappop(self.queue)
            if rid not in self.killed:
                self.nodes[rid].receive(message)
        for rid, node in sorted(self.nodes.items()):
            if rid not in self.killed:
                node.tick()
        if self.mode == 'keep_owner':
            if execute:
                self.baseline_execute(duration)
            return
        for tid in self.nodes[next(iter(self.nodes))].tasks:
            owners = [node for rid, node in self.nodes.items()
                      if rid not in self.killed and node.may_execute(tid)]
            if len(owners) > 1:
                self.duplicate_assignments += 1
                raise AssertionError(f'duplicate executable ownership: {tid}')
            if owners and execute:
                owner = owners[0]
                token = owner.ledger.entries[tid]['token']
                key = (tid, token)
                self.execution_ticks[key] = self.execution_ticks.get(key, 0) + duration
                # Abstract executor (route length / speed), NOT a physical robot simulation.
                need = self.task_seconds(owner.ledger.entries[tid].get('route'))
                if self.execution_ticks[key] >= need and owner.complete(tid):
                    self.completed.add(tid)
                    self.completed_at[tid] = self.time

    def run(self, seconds, execute=False):
        end = self.time + seconds
        while self.time < end:
            self.step(execute=execute)
        return self

    def warmup(self):
        return self.run(.3)

    def kill(self, rid):
        self.killed.add(rid)
        self.events.append(dict(t=self.time, robot=rid, kind='fault_kill'))

    def snapshot(self):
        return dict(time=self.time, nodes=[n.snapshot() for n in self.nodes.values()],
                    events=self.events[-300:], metrics=self.metrics())

    def metrics(self):
        announces = {}
        confirms, commits = {}, {}
        protocol_messages = sum(e['kind'].startswith('send_') and e['kind'] != 'send_heartbeat'
                                for e in self.events)
        for e in self.events:
            tid = e.get('task_id')
            if e['kind'] == 'send_announce':
                announces.setdefault(tid, e['t'])
            if e['kind'] == 'send_confirm' and e.get('accepted'):
                confirms.setdefault(tid, e['t'])
            if e['kind'] == 'ownership_committed':
                commits.setdefault(tid, e['t'])
        tasks = self.config['tasks']
        known = set().union(*(set(n.tasks) for n in self.nodes.values()))
        final_owner = {}
        for e in self.events:
            if e['kind'] == 'ownership_committed':
                final_owner[e['task_id']] = e['owner']
        return dict(mode=self.mode, completed=len(self.completed), task_count=len(tasks),
                    completion_time_s={t: round(v - self.task_start, 2) for t, v in self.completed_at.items()},
                    final_owner=final_owner,
                    messages_per_auction=self.auction_transmissions,
                    completion_percent=100 * len(self.completed) / max(1, len(tasks)),
                    transmissions_including_retries_and_heartbeats=self.messages,
                    new_protocol_messages_excluding_retries=protocol_messages,
                    announce_to_confirm_ms={t: round((v - announces[t]) * 1000, 2)
                                            for t, v in confirms.items() if t in announces},
                    announce_to_commit_ms={t: round((v - announces[t]) * 1000, 2)
                                           for t, v in commits.items() if t in announces},
                    duplicate_execution_observations=self.duplicate_assignments,
                    tasks_missing_from_all_ledgers=len({t['task_id'] for t in tasks} - known))

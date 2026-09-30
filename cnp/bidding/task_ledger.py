"""Durable, fail-closed quorum voters. Static, trusted fleet membership."""
import json
import os
from pathlib import Path

class Ledger:
    def __init__(self, members, skew, path=None):
        self.members = set(members)
        self.majority = len(members) // 2 + 1
        self.skew = skew
        self.path = Path(path) if path else None
        self.votes, self.entries, self.epochs = {}, {}, {}
        if self.path and self.path.exists():
            data = json.loads(self.path.read_text())
            self.votes = data['votes']
            self.entries = data['entries']
            self.epochs = data['epochs']

    def save(self):
        if not self.path:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        with tmp.open('w') as out:
            json.dump({'votes': self.votes, 'entries': self.entries, 'epochs': self.epochs}, out)
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, self.path)
        if hasattr(os, 'O_DIRECTORY'):
            fd = os.open(self.path.parent, os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)

    def next_epoch(self, task_id):
        self.epochs[task_id] = self.epochs.get(task_id, 0) + 1
        self.save()
        return self.epochs[task_id]

    def observe_epoch(self, task_id, epoch):
        self.epochs[task_id] = max(epoch, self.epochs.get(task_id, 0))

    def grant(self, proposal, now):
        tid = proposal['task_id']
        if proposal['owner'] not in self.members or proposal['lease_end'] <= now + self.skew:
            return False
        entry = self.entries.get(tid, {})
        if entry.get('status') == 'done':
            return False
        old = self.votes.get(tid)
        if old and old['token'] == proposal['token']:
            comparable = {k: v for k, v in old.items() if k != 'lease_end'}
            incoming = {k: v for k, v in proposal.items() if k != 'lease_end'}
            if comparable != incoming or proposal['lease_end'] < old['lease_end']:
                return False
            self.votes[tid] = dict(proposal)
            self.save()
            return True
        # Even a higher epoch cannot override an unexpired lease.
        if old and now <= old['lease_end'] + self.skew:
            return False
        if entry and entry.get('lease_end', 0) + self.skew >= now and entry.get('token') != proposal['token']:
            return False
        self.votes[tid] = dict(proposal)
        self.observe_epoch(tid, proposal['epoch'])
        self.save()  # Must be durable BEFORE the grant is sent.
        return True

    def commit(self, proposal, voters, now):
        if len(set(voters) & self.members) < self.majority:
            return False
        if proposal['lease_end'] <= now + self.skew:
            return False
        tid = proposal['task_id']
        old = self.entries.get(tid, {})
        if old.get('status') == 'done':
            return False
        if old and old.get('token') != proposal['token'] and old.get('lease_end', 0) + self.skew >= now:
            return False
        if old.get('token') == proposal['token'] and old.get('lease_end', 0) > proposal['lease_end']:
            return False
        self.entries[tid] = dict(proposal, status='owned', voters=sorted(set(voters)))
        self.observe_epoch(tid, proposal['epoch'])
        self.save()
        return True

    def can_execute(self, task_id, owner, now):
        entry = self.entries.get(task_id, {})
        return (entry.get('status') == 'owned' and entry.get('owner') == owner
                and now < entry['lease_end'] - self.skew)

    def done(self, task_id, token):
        entry = self.entries.get(task_id)
        if entry and entry['token'] == token:
            entry['status'] = 'done'
            self.save()
            return True
        return False

    def merge_done(self, entry):
        # Trusted crash-fault peers gossip permanent terminal tombstones.
        if entry.get('status') != 'done' or len(set(entry.get('voters', [])) & self.members) < self.majority:
            return False
        self.entries[entry['task_id']] = dict(entry)
        self.save()
        return True

"""All auction messages include the complete auction identity."""
import json
import uuid
from dataclasses import asdict, dataclass

@dataclass(frozen=True)
class Message:
    kind: str
    sender: str
    timestamp: float
    payload: dict
    message_id: str

    @property
    def topic(self):
        if self.kind == "announce":
            return "cnp/announce"
        if self.kind == "heartbeat":
            return f"fleet/{self.sender}/status"
        if self.kind == "blocked_edges":
            return "map/blocked_edges"
        return f"cnp/{self.payload['task_id']}/{self.kind}"

    def encode(self):
        return json.dumps(asdict(self), sort_keys=True, allow_nan=False).encode()

    @classmethod
    def decode(cls, data):
        return cls(**json.loads(data))

    @classmethod
    def make(cls, kind, sender, timestamp, payload, message_id=None):
        return cls(kind, sender, timestamp, payload, message_id or uuid.uuid4().hex)

CORE_KINDS = {"announce", "bid", "award", "confirm", "cancel", "decline"}
CONTROL_KINDS = {"lease_request", "lease_grant", "commit", "release", "heartbeat", "blocked_edges"}

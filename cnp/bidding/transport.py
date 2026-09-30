"""Zenoh callbacks enqueue only: Node is deliberately single-threaded."""
from queue import Queue, Empty
from .messages import Message

class ZenohTransport:
    def __init__(self, config_path=None):
        import zenoh
        config = zenoh.Config.from_file(config_path) if config_path else zenoh.Config()
        self.session = zenoh.open(config)
        self.queue = Queue()
        self.errors = []
        self.subscribers = [self.session.declare_subscriber(topic, self._receive)
                            for topic in ('cnp/**', 'fleet/*/status', 'map/blocked_edges')]

    def _receive(self, sample):
        try:
            data = sample.payload.to_bytes()
            if len(data) > 1000000:
                raise ValueError('message exceeds size limit')
            self.queue.put(Message.decode(data))
        except Exception as exc:
            self.errors.append(str(exc))
            self.errors[:] = self.errors[-100:]

    def send(self, message):
        self.session.put(message.topic, message.encode())

    def drain(self, node):
        for _ in range(1000):
            try:
                message = self.queue.get_nowait()
            except Empty:
                break
            try:
                node.receive(message)
            except (KeyError, TypeError, ValueError) as exc:
                node.event('invalid_message', error=str(exc))

    def close(self):
        self.session.close()

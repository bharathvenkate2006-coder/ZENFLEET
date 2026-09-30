"""Local-only fault-injection server. Never expose this endpoint to a fleet network."""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from .demo_scenarios import prepare, SCENARIOS

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    lock = threading.Lock()
    box = [prepare('basic', config)]
    html = Path('dashboard/index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, code, body, content_type='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                return self.respond(200, html, 'text/html; charset=utf-8')
            if self.path != '/state':
                return self.respond(404, b'{}')
            with lock:
                data = dict(box[0].snapshot(), scenarios=SCENARIOS)
            self.respond(200, json.dumps(data).encode())

        def do_POST(self):
            if self.path != '/control' or self.headers.get('Content-Type') != 'application/json':
                return self.respond(400, b'{}')
            origin = self.headers.get('Origin')
            if origin and origin != f'http://{self.headers.get("Host")}':
                return self.respond(403, b'{}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length < 8192:
                    raise ValueError('invalid body size')
                p = json.loads(self.rfile.read(length))
                with lock:
                    sim = box[0]
                    node = sim.nodes[p.get('robot', 'R1')]
                    action = p['action']
                    if action == 'reset':
                        if p['scenario'] not in SCENARIOS:
                            raise ValueError('unknown scenario')
                        box[0] = prepare(p['scenario'], config)
                    elif action in {'block', 'clear'}:
                        node.set_block('A', 'B', action == 'block')
                        if action == 'clear':
                            node.cancel('T1')
                    elif action == 'kill':
                        sim.kill(node.id)
                    elif action == 'drain':
                        node.robot['battery'] = 0
                    elif action == 'split':
                        sim.partitions = [{'R1'}, {'R2', 'R3'}]
                    elif action == 'heal':
                        sim.partitions = []
                    elif action == 'network':
                        sim.drop = min(1, max(0, float(p['drop'])))
                        sim.delay = min(5, max(0, float(p['delay'])))
                    else:
                        raise ValueError('unknown action')
                self.respond(200, b'{"ok":true}')
            except (KeyError, ValueError, TypeError) as exc:
                self.respond(400, json.dumps({'error': str(exc)}).encode())

    stop = threading.Event()
    def advance():
        while not stop.wait(.02):
            with lock:
                box[0].step(execute=True)
    worker = threading.Thread(target=advance, daemon=True)
    worker.start()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(f'Open http://127.0.0.1:{args.port}; Ctrl+C stops the demo')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        worker.join()

if __name__ == '__main__':
    main()

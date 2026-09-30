"""Real Zenoh protocol runner. This does not actuate a robot."""
import argparse
import json
import time
from pathlib import Path
from .initiator import Node
from .transport import ZenohTransport

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True)
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--zenoh-config')
    parser.add_argument('--start-task')
    parser.add_argument('--state-dir', default='state')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if args.robot not in config['members']:
        parser.error('robot must be a configured member')
    state = Path(args.state_dir)
    state.mkdir(parents=True, exist_ok=True)
    transport = ZenohTransport(args.zenoh_config)
    try:
        with (state / f'{args.robot}.jsonl').open('a', buffering=1) as log:
            node = Node(args.robot, config, transport.send, time.time,
                        lambda e: log.write(json.dumps(e) + '\n'),
                        state / f'{args.robot}-ledger.json')
            started, boot, previous = False, time.time(), time.time()
            print(f'{args.robot}: protocol only; execution requires a fenced driver adapter.')
            while True:
                now = time.time()
                if now < previous:
                    raise RuntimeError('wall clock moved backwards; fail closed and stop driver')
                previous = now
                transport.drain(node)
                node.tick()
                if args.start_task and not started and now - boot > 2:
                    node.start(args.start_task, 'operator_demo')
                    started = True
                snapshot = state / f'{args.robot}-snapshot.json'
                tmp = snapshot.with_suffix('.tmp')
                tmp.write_text(json.dumps(node.snapshot(), indent=2))
                tmp.replace(snapshot)
                time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()

if __name__ == '__main__':
    main()

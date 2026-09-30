import argparse
import copy
import json
from pathlib import Path
from .simulator import Simulation
from .safety_bridge import apply_safety_output

SCENARIOS = ['basic', 'dead_robot', 'no_bidders', 'tie', 'no_confirm',
             'initiator_dies', 'two_auctions', 'loss_late_duplicate', 'network_split',
             'cancel', 'battery_drop', 'cargo_rescue', 'all_busy', 'loop_limit',
             'junction_congestion', 'low_range', 'safety_fallback']

def prepare(name, config, seed=None, enabled=True, mode=None):
    cfg = copy.deepcopy(config)
    if name == 'two_auctions':
        cfg['tasks'].append(dict(cfg['tasks'][0], task_id='T2', priority=20))
    sim = Simulation(cfg, seed, enabled, mode)
    sim.warmup()
    sim.task_start = sim.time
    n = sim.nodes
    baseline = sim.mode == 'keep_owner'
    if name == 'no_bidders':
        for node in n.values():
            node.robot['battery'] = 0
    if name == 'tie':
        for node in n.values():
            node.robot.update(node='A', battery=80)
    if name == 'all_busy':
        for i, node in enumerate(n.values()):
            node.robot['queue_load'] = i + 1
    if name == 'cargo_rescue':
        for node in n.values():
            node.tasks['T1']['carrying_cargo'] = True
    if name == 'loop_limit':
        n['R1'].reassignments['T1'] = cfg['reassignment_limit']
    if name == 'loss_late_duplicate':
        sim.drop, sim.delay, sim.duplicate = .15, .3, .6
    if name == 'network_split':
        sim.partitions = [{'R1'}, {'R2', 'R3'}]
        n['R2'].start('T1', 'peer_lost')
    if name == 'junction_congestion':
        n['R1'].robot['fault'] = 'motor_fault'   # the initiator cannot take the job itself
    if name == 'low_range':
        n['R3'].robot['battery'] = 11            # nearest robot, but not enough charge left
    if name == 'basic':
        n['R1'].set_block('A', 'B')
        sim.run(.15)
    n['R1'].start('T1')
    if name == 'two_auctions':
        n['R1'].start('T2')
    if name == 'cancel':
        sim.run(.12)
        n['R1'].cancel('T1')
    if name == 'no_confirm':
        # Best bidder R1 disappears before it processes the award.
        sim.run(.47)
        sim.kill('R1')
    if name == 'initiator_dies':
        sim.run(.2)
        sim.kill('R1')
    if name in {'dead_robot', 'battery_drop', 'safety_fallback'}:
        sim.run(1)
        # whoever is executing the task right now gets hit (the original owner in the baseline)
        owners = ['R1'] if baseline else [r for r, node in n.items() if node.may_execute('T1')]
        if owners:
            if name == 'dead_robot':
                sim.kill(owners[0])
            elif name == 'battery_drop':
                n[owners[0]].robot['battery'] = 0
            else:
                # the ZenFleet speed controller reports a fallback (stale peer data) that never clears
                apply_safety_output(n[owners[0]], dict(fallback_flag=1, reasons=['stale_or_lost_data']))
    if name == 'network_split':
        sim.run(1.5)
        sim.partitions = []
    return sim

def run_scenario(name, config, seed=None, enabled=True, seconds=None, mode=None):
    sim = prepare(name, config, seed, enabled, mode)
    sim.run(seconds if seconds is not None else config.get('sim_seconds', 12), execute=True)
    return sim

def print_table(results):
    rows = {}
    for r in results:
        rows.setdefault((r['scenario'], r['seed']), {})[r['mode']] = r
    print(f"{'scenario':22s} {'no-reassignment baseline':26s} {'CNP':26s} winner")
    for (name, seed), by in rows.items():
        cells = []
        for mode in ('keep_owner', 'cnp'):
            r = by.get(mode)
            if r is None:
                cells.append('-'.ljust(26))
            elif r['completed']:
                t = max(r['completion_time_s'].values())
                cells.append(f"done in {t:5.1f} s".ljust(26))
            else:
                cells.append('not completed'.ljust(26))
        winner = ', '.join(sorted(set(by['cnp']['final_owner'].values()))) if 'cnp' in by else ''
        print(f"{name:22s} {cells[0]} {cells[1]} {winner}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--scenario', choices=SCENARIOS + ['all'], default='basic')
    parser.add_argument('--seed', type=int)
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--compare', action='store_true',
                        help='also run the no-reassignment baseline (original owner keeps the task)')
    parser.add_argument('--json', action='store_true', help='print full JSON instead of the table')
    parser.add_argument('--out', default='runs')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for name in SCENARIOS if args.scenario == 'all' else [args.scenario]:
        for i in range(args.runs):
            seed = (args.seed if args.seed is not None else config['seed']) + i
            for mode in (['keep_owner', 'cnp'] if args.compare else ['cnp']):
                sim = run_scenario(name, config, seed, mode=mode)
                label = f'{name}-{seed}-{mode}'
                (output / f'{label}.json').write_text(json.dumps(sim.snapshot(), indent=2))
                with (output / f'{label}.jsonl').open('w') as stream:
                    for event in sim.events:
                        stream.write(json.dumps(event) + '\n')
                results.append(dict(scenario=name, seed=seed, cnp=(mode == 'cnp'), **sim.metrics()))
    (output / 'summary.json').write_text(json.dumps(results, indent=2))
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print_table(results)
        print(f"\nFull results: {output / 'summary.json'}")

if __name__ == '__main__':
    main()

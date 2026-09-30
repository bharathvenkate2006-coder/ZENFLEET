"""
Worked examples: run a few scenarios on the ZenFleet warehouse map and narrate what happens.

    python examples/run_examples.py            # from the cnp/ folder

Everything here uses the trained ZenFleet models (models/ in the repository root).
"""
import json
import sys
import warnings
from pathlib import Path

warnings.filterwarnings('ignore')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bidding.demo_scenarios import run_scenario          # noqa: E402
from bidding.predictors import make_predictor            # noqa: E402

CFG = json.loads((ROOT / 'config_zenfleet.json').read_text())

MAP = """
    A ---4--- B ---4--- C          B and E are the shared junctions.
    |         |         |          Numbers are edge lengths in metres.
    3         3         3
    |         |         |
    D ---4--- E ---4--- F
"""


def show_bids(sim):
    seen = set()
    for e in sim.events:
        if e['kind'] == 'send_bid' and e['robot'] not in seen:
            seen.add(e['robot'])
            b = e['cost_breakdown']
            print(f"   {e['robot']} bids {e['total_cost'] / 1e6:6.2f}  "
                  f"(trip {b['travel']:.1f} s, junction delay {b['congestion']:.1f} s, "
                  f"range left {e['range_remaining_m']:.0f} m, conflict risk {e['max_conflict_risk']:.2f})")
            for d in e['junction_detail']:
                print(f"        junction {d['junction']}: {d['peer']} is coming too. Q50 clear {d['t_clear_q50']} s, "
                      f"Q90 clear {d['t_clear_q90']} s, wait {d['wait_s']} s")
        if e['kind'] == 'send_decline' and e['robot'] not in seen:
            seen.add(e['robot'])
            print(f"   {e['robot']} declines: {e['reason']}")


def timeline(sim):
    keep = {'fault_kill': 'robot killed', 'safety_fallback': 'safety layer reports fallback',
            'send_announce': 'announces task', 'send_award': 'awards task',
            'completed_locally': 'finishes the task'}
    printed = set()
    for e in sim.events:
        if e['kind'] in keep:
            extra = e.get('reason') or (f"to {e['winner_id']}" if e['kind'] == 'send_award' else '')
            line = (round(e['t'], 2), e['robot'], keep[e['kind']], extra)
            if line not in printed:
                printed.add(line)
                print(f"   t={line[0]:5.2f}s  {line[1]}  {line[2]} {line[3]}".rstrip())


def run(title, name, what_to_see, **kw):
    print('=' * 78)
    print(title)
    print(what_to_see)
    sim = run_scenario(name, CFG, mode=kw.get('mode', 'cnp'))
    return sim


print(MAP)

sim = run('Example 1: the nearest robot would run into traffic', 'junction_congestion',
          'R1 has a motor fault, so R2 and R3 bid. R3 is a hair cheaper on distance alone,\n'
          'but R2 is about to drive through junction E.')
show_bids(sim)
m = sim.metrics()
print(f"   -> winner {m['final_owner']['T1']}, task done in {m['completion_time_s']['T1']} s")

from copy import deepcopy                                # noqa: E402
off = deepcopy(CFG)
off['predictor']['use_congestion'] = False
plain = run_scenario('junction_congestion', off)
print(f"   Same situation with the junction prediction switched off: winner "
      f"{plain.metrics()['final_owner']['T1']} (its route crosses R2 at junction E, so it would have to wait there).")

sim = run('\nExample 2: a robot that is close but nearly out of battery', 'low_range',
          'R3 is right next to the pickup but has 11% battery, with a 10% reserve.')
show_bids(sim)
m = sim.metrics()
print(f"   -> winner {m['final_owner']['T1']}, task done in {m['completion_time_s']['T1']} s")

sim = run('\nExample 3: the speed controller reports a fallback that does not clear', 'safety_fallback',
          'The owner loses fresh peer data. A short glitch is ignored; this one lasts.')
timeline(sim)
m = sim.metrics()
print(f"   -> new owner {m['final_owner']['T1']}, task done in {m['completion_time_s']['T1']} s "
      f"(no duplicate execution: {m['duplicate_execution_observations'] == 0})")

sim = run('\nExample 4: the robot holding the task dies', 'dead_robot',
          "Its peers notice the missing heartbeat and auction the task themselves.")
timeline(sim)
m = sim.metrics()
base = run_scenario('dead_robot', CFG, mode='keep_owner').metrics()
print(f"   -> CNP: done in {m['completion_time_s']['T1']} s by {m['final_owner']['T1']}. "
      f"Without reassignment: {'done' if base['completed'] else 'never finished'}.")

print('\nPrediction source used for the bids above:', make_predictor(CFG).source)

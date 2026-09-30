import copy
import json
from pathlib import Path
import pytest
from bidding.demo_scenarios import run_scenario, prepare
from bidding.cost import evaluate, route, bid_key
from bidding.task_ledger import Ledger
from bidding.messages import Message

@pytest.fixture
def config():
    return json.loads(Path('config.json').read_text())

def kinds(sim):
    return {event['kind'] for event in sim.events}

def committed_owners(sim):
    return {e['owner'] for e in sim.events if e['kind'] == 'ownership_committed'}

def test_basic(config):
    sim = prepare('basic', config)
    sim.nodes['R1'].set_block('A', 'B')
    sim.run(12, execute=True)
    assert sim.completed == {'T1'}
    assert 'R1' in committed_owners(sim)
    assert route(config['graph'], 'A', 'C', {'A|B'})[1] == ['A', 'D', 'C']

def test_dead_robot(config):
    sim = run_scenario('dead_robot', config)
    assert sim.completed == {'T1'}
    assert len(committed_owners(sim)) >= 2
    assert any(e.get('reason') == 'owner_lost_or_lease_expired' for e in sim.events)

def test_no_bidders(config):
    sim = run_scenario('no_bidders', config)
    assert not sim.completed
    assert 'unassigned' in kinds(sim)
    assert max(e['radius'] for e in sim.events if e['kind'] == 'send_announce') > config['announcement_radius']

def test_tie(config):
    sim = run_scenario('tie', config)
    assert committed_owners(sim) == {'R1'}
    assert sim.completed == {'T1'}

def test_winner_never_confirms(config):
    sim = run_scenario('no_confirm', config)
    assert sim.completed == {'T1'}
    assert 'R1' not in committed_owners(sim)

def test_initiator_dies(config):
    sim = run_scenario('initiator_dies', config)
    assert sim.completed == {'T1'}
    assert any(e['kind'] == 'send_award' and e['robot'] != 'R1' for e in sim.events)

def test_two_auctions(config):
    sim = run_scenario('two_auctions', config)
    assert sim.completed == {'T1', 'T2'}
    assert sim.duplicate_assignments == 0
    first = next(e for e in sim.events if e['kind'] == 'send_confirm' and e.get('accepted') and e['robot'] == 'R1')
    assert first['task_id'] == 'T2'

def test_loss_late_duplicate(config):
    sim = run_scenario('loss_late_duplicate', config, seconds=20)
    assert sim.duplicate_assignments == 0
    assert sim.metrics()['tasks_missing_from_all_ledgers'] == 0
    assert any(e['kind'] == 'send_announce' for e in sim.events)
    # Unbounded packet loss cannot guarantee completion; test liveness after healing.
    sim.drop, sim.delay, sim.duplicate = 0, .015, 0
    for node in sim.nodes.values():
        a = node.auctions.get('T1')
        if a and a['state'] == 'FAILED':
            node.start('T1', 'network_healed')
            break
    sim.run(20, execute=True)
    assert sim.completed == {'T1'}

def test_network_split(config):
    sim = run_scenario('network_split', config)
    assert sim.completed == {'T1'}
    assert sim.duplicate_assignments == 0

def test_cancel(config):
    sim = run_scenario('cancel', config)
    assert not sim.completed
    assert 'cancelled' in kinds(sim)

def test_battery_drop(config):
    sim = run_scenario('battery_drop', config)
    assert sim.completed == {'T1'}
    assert any(e.get('reason') == 'battery_low' for e in sim.events)

def test_cargo_rescue(config):
    sim = run_scenario('cargo_rescue', config)
    assert not sim.completed
    assert 'rescue_required' in kinds(sim)

def test_all_busy(config):
    sim = run_scenario('all_busy', config)
    assert sim.completed == {'T1'}
    bids = [e for e in sim.events if e['kind'] == 'send_bid']
    assert bids and all(e['cost_breakdown']['queue'] > 0 for e in bids)

def test_reassignment_limit(config):
    sim = run_scenario('loop_limit', config)
    assert not sim.completed
    assert 'human_required' in kinds(sim)

def test_deterministic_seed(config):
    a = run_scenario('basic', config)
    b = run_scenario('basic', config)
    assert a.events == b.events

def test_persistent_vote_and_expiry(tmp_path):
    path = tmp_path / 'ledger.json'
    ledger = Ledger(['R1', 'R2', 'R3'], .05, path)
    p = dict(task_id='T1', owner='R1', token='a', epoch=1, lease_end=3)
    assert ledger.grant(p, 0)
    recovered = Ledger(['R1', 'R2', 'R3'], .05, path)
    other = dict(p, owner='R2', token='b', epoch=999, lease_end=6)
    assert not recovered.grant(other, 1)
    assert recovered.grant(other, 3.1)

def test_quorum_not_epoch_alone():
    ledger = Ledger(['R1', 'R2', 'R3'], .05)
    p = dict(task_id='T1', owner='R1', token='a', epoch=1, lease_end=3)
    assert not ledger.commit(p, ['R1'], 0)
    assert ledger.commit(p, ['R1', 'R2'], 0)
    assert ledger.can_execute('T1', 'R1', 2.9)
    assert not ledger.can_execute('T1', 'R1', 2.96)

def test_eligibility(config):
    robot = copy.deepcopy(config['robots']['R1'])
    task = config['tasks'][0]
    for change in ({'battery': 0}, {'capacity': 0}, {'critical': True},
                   {'localization_ok': False}, {'comms_ok': False}, {'telemetry_valid_until': -1}):
        assert evaluate(dict(robot, **change), task, config['graph'], set(), config, 0)[0] is None

def test_message_roundtrip():
    m = Message.make('bid', 'R1', 1.0, {'task_id': 'T1', 'key': [1, 'R2']}, 'fixed')
    assert Message.decode(m.encode()) == m
    assert m.topic == 'cnp/T1/bid'


def test_late_bid_ignored(config):
    from bidding.simulator import Simulation
    sim = Simulation(config).warmup()
    sim.nodes['R1'].start('T1')
    sim.run(.15)
    node = sim.nodes['R1']
    a = node.auctions['T1']
    original = copy.deepcopy(a['bids'])
    sim.time = a['closes'] + .01
    bid = dict(task_id='T1', epoch=a['epoch'], key=a['key'], bidder_id='R2',
               total_cost=-1, battery=100)
    node.receive(Message.make('bid', 'R2', sim.time, bid, 'late'))
    assert a['bids'] == original

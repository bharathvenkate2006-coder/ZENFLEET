"""Tests for the ZenFleet prediction-model integration (needs lightgbm, pandas, scikit-learn, joblib)."""
import copy
import json
from pathlib import Path

import pytest

pytest.importorskip('lightgbm')
pytest.importorskip('pandas')
pytest.importorskip('sklearn')
pytest.importorskip('joblib')

from bidding.demo_scenarios import SCENARIOS, prepare, run_scenario
from bidding.messages import Message
from bidding.simulator import Simulation
from bidding.predictors import PredictionAdapter, make_predictor
from bidding.safety_bridge import apply_safety_output


@pytest.fixture
def cfg():
    return json.loads(Path('config_zenfleet.json').read_text())


def declines(sim):
    return {e['robot']: e['reason'] for e in sim.events if e['kind'] == 'send_decline'}


def first_bids(sim):
    out = {}
    for e in sim.events:
        if e['kind'] == 'send_bid':
            out.setdefault(e['robot'], e)
    return out


# ---------------------------------------------------------------- the adapter
def test_trained_models_load(cfg):
    adapter = make_predictor(cfg)
    assert adapter.source == 'lightgbm', adapter.model_error


def test_range_shrinks_with_payload_and_battery(cfg):
    a = PredictionAdapter(cfg)
    robot = dict(cfg['robots']['R3'])
    assert a.range_m(robot, 0) > a.range_m(robot, 30)
    assert a.energy_per_m(robot, 30) > a.energy_per_m(robot, 0)
    assert a.range_m(dict(robot, battery=10), 2) == 0          # at the reserve line
    assert a.range_m(dict(robot, battery=11), 2) < 10           # a sliver above it


def peers_for(route, speed=0.8):
    return {'R2': dict(node=route[0], speed=speed, route=route, priority=1, seen=0.0)}


def test_junction_prediction_is_sane_and_reacts_to_timing(cfg):
    a = PredictionAdapter(cfg)
    me = dict(cfg['robots']['R3'])
    route = ['D', 'E', 'F']                       # I reach junction E after 4 m
    close = a.junction_delay('R3', me, route, peers_for(['F', 'E', 'D']), 0.1, 2)
    far = a.junction_delay('R3', me, route, peers_for(['C', 'F', 'E', 'D']), 0.1, 2)
    assert close['junctions'] and far['junctions']
    for j in close['junctions'] + far['junctions']:
        assert 0 <= j['conflict_risk'] <= 1
        assert j['t_clear_q90'] >= j['t_clear_q50']
    assert close['max_risk'] > far['max_risk']    # a peer arriving with me is riskier than one far behind
    assert close['expected_delay_s'] > far['expected_delay_s']


def test_no_shared_junction_means_no_delay(cfg):
    a = PredictionAdapter(cfg)
    out = a.junction_delay('R3', dict(cfg['robots']['R3']), ['D', 'A'], peers_for(['F', 'E', 'D']), 0.1, 2)
    assert out['junctions'] == [] and out['expected_delay_s'] == 0


def test_stale_peers_are_ignored(cfg):
    a = PredictionAdapter(cfg)
    stale = {'R2': dict(node='F', speed=.8, route=['F', 'E', 'D'], priority=1, seen=0.0)}
    out = a.junction_delay('R3', dict(cfg['robots']['R3']), ['D', 'E', 'F'], stale, now=5.0, payload=2)
    assert out['junctions'] == []


def test_physics_fallback_when_models_disabled(cfg):
    c = copy.deepcopy(cfg)
    c['predictor']['use_models'] = False
    sim = run_scenario('junction_congestion', c)
    bids = first_bids(sim)
    assert bids and all(b['prediction_source'] == 'physics' for b in bids.values())
    assert sim.completed == {'T1'}


# ------------------------------------------------------- bids in the auction
def test_junction_delay_changes_the_winner(cfg):
    with_delay = run_scenario('junction_congestion', cfg)
    off = copy.deepcopy(cfg)
    off['predictor']['use_congestion'] = False
    without = run_scenario('junction_congestion', off)
    assert with_delay.metrics()['final_owner'] == {'T1': 'R2'}
    assert without.metrics()['final_owner'] == {'T1': 'R3'}
    bid = first_bids(with_delay)['R3']
    assert bid['cost_breakdown']['congestion'] > 0
    assert bid['max_conflict_risk'] > 0.5
    assert bid['junction_detail'][0]['peer'] == 'R2'


def test_low_range_robot_declines_and_next_robot_wins(cfg):
    sim = run_scenario('low_range', cfg)
    assert declines(sim).get('R3') == 'insufficient_range'
    assert sim.metrics()['final_owner'] == {'T1': 'R2'}
    assert sim.completed == {'T1'}


# ------------------------------------------------------- safety layer bridge
def test_long_safety_fallback_hands_the_task_over(cfg):
    sim = run_scenario('safety_fallback', cfg)
    fell_back = next(e['robot'] for e in sim.events if e['kind'] == 'safety_fallback')
    assert any(e['kind'] == 'send_announce' and e.get('reason') in {'safety_fallback', 'robot_fault'}
               for e in sim.events)
    assert sim.completed == {'T1'}
    assert sim.metrics()['final_owner']['T1'] != fell_back
    assert sim.duplicate_assignments == 0


def test_short_glitch_does_not_trigger_an_auction(cfg):
    sim = prepare('basic', cfg, mode='cnp')
    sim.run(1.2)
    owner = next(r for r, n in sim.nodes.items() if n.may_execute('T1'))
    node = sim.nodes[owner]
    apply_safety_output(node, dict(fallback_flag=1, reasons=['stale_or_lost_data']))
    sim.run(1.0)                                              # shorter than fallback_timeout (2 s)
    apply_safety_output(node, dict(fallback_flag=0, reasons=[]))
    sim.run(30, execute=True)
    assert not any(e.get('reason') in {'safety_fallback', 'robot_fault'} for e in sim.events
                   if e['kind'] == 'send_announce')
    assert sim.completed == {'T1'}


def test_safety_bridge_flags_blocked_edge_without_detour(cfg):
    sim = Simulation(cfg, mode='cnp').warmup()
    node = sim.nodes['R2']
    apply_safety_output(node, dict(fallback_flag=0, reasons=['blocked_edge']))
    assert node.robot['blocked_no_detour'] is True
    assert node.robot['safety_fallback_since'] is not None
    apply_safety_output(node, dict(fallback_flag=0, reasons=[]))
    assert node.robot['blocked_no_detour'] is False and node.robot['safety_fallback_since'] is None


def test_fallback_robot_does_not_bid_for_new_work(cfg):
    sim = Simulation(cfg, mode='cnp').warmup()
    apply_safety_output(sim.nodes['R3'], dict(fallback_flag=1, reasons=['stale_or_lost_data']))
    sim.nodes['R1'].start('T1', 'operator_demo')
    sim.run(1.5)
    assert declines(sim).get('R3') == 'safety_fallback'


# ------------------------------------------------------------- all scenarios
EXPECTED_DONE = {
    'basic': True, 'dead_robot': True, 'no_bidders': False, 'tie': True, 'no_confirm': True,
    'initiator_dies': True, 'two_auctions': True, 'loss_late_duplicate': True,
    'network_split': True, 'cancel': False, 'battery_drop': True, 'cargo_rescue': False,
    'all_busy': True, 'loop_limit': False, 'junction_congestion': True, 'low_range': True,
    'safety_fallback': True,
}


def test_every_scenario_is_listed():
    assert set(EXPECTED_DONE) == set(SCENARIOS)


@pytest.mark.parametrize('name', SCENARIOS)
def test_scenario_outcome_and_safety(cfg, name):
    sim = run_scenario(name, cfg, mode='cnp')
    m = sim.metrics()
    assert m['duplicate_execution_observations'] == 0
    assert m['tasks_missing_from_all_ledgers'] == 0
    done = m['completed'] == m['task_count']
    assert done == EXPECTED_DONE[name]


@pytest.mark.parametrize('name', ['dead_robot', 'battery_drop', 'safety_fallback',
                                  'initiator_dies', 'junction_congestion'])
def test_reassignment_beats_keeping_the_task(cfg, name):
    """When the robot holding the task is broken, keeping it means the task is never finished."""
    base = run_scenario(name, cfg, mode='keep_owner')
    cnp = run_scenario(name, cfg, mode='cnp')
    assert base.metrics()['completed'] == 0
    assert cnp.metrics()['completed'] == 1


def test_healthy_owner_is_not_slower_by_more_than_the_handover(cfg):
    """Honest check: with a healthy nearby owner, CNP should cost roughly one handover, no more."""
    base = run_scenario('tie', cfg, mode='keep_owner').metrics()['completion_time_s']['T1']
    cnp = run_scenario('tie', cfg, mode='cnp').metrics()['completion_time_s']['T1']
    assert cnp <= base + 2.0


# ------------------------------------------------------------------- naming
def test_status_topic_matches_fleet_naming():
    m = Message.make('heartbeat', 'R2', 1.0, {'node': 'F'}, 'x')
    assert m.topic == 'fleet/R2/status'
    assert Message.make('blocked_edges', 'R1', 1.0, {'edge': 'A|B'}, 'y').topic == 'map/blocked_edges'

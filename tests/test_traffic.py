"""Tests for the multi-robot traffic simulation (traffic/)."""
import sys
import warnings
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('lightgbm')
warnings.filterwarnings('ignore')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'src'))

from traffic.control import RAW, batch_step, load_controller   # noqa: E402
from traffic.sim import Params, Scenario, run                    # noqa: E402

INT_KEYS = ('peer_intent', 'lease_status', 'priority_diff', 'heartbeat_ok', 'blocked_edge')


@pytest.fixture(scope='module')
def ctrl():
    return load_controller()


def random_states(n, seed=0):
    rng = np.random.default_rng(seed)
    return dict(
        v_ego=rng.uniform(0, 1, n), d_ego=rng.uniform(0.3, 8, n), v_peer=rng.uniform(0, 1, n),
        d_peer=rng.uniform(-1, 8, n), peer_intent=(rng.random(n) < .85).astype(int),
        lease_status=rng.choice([0, 1, 2], n), priority_diff=rng.integers(-3, 4, n),
        payload_kg=rng.choice([0, 5, 10, 20, 30], n).astype(float), msg_age_ms=rng.uniform(0, 2000, n),
        a_max=rng.uniform(0.5, 1.1, n), robot_len=np.full(n, 0.8), d_safe=np.full(n, 0.3),
        loc_age_s=rng.uniform(0, 7, n), heartbeat_ok=(rng.random(n) > .05).astype(int),
        blocked_edge=(rng.random(n) < .03).astype(int)), rng.uniform(0.4, 1.0, n)


def test_batch_controller_matches_single_robot_controller(ctrl):
    """The vectorised copy must give the same speed command as SpeedController.step."""
    st, vp = random_states(300)
    out = batch_step(ctrl, st, vp, np.ones(300, bool))
    for i in range(300):
        s = {k: (int(st[k][i]) if k in INT_KEYS else float(st[k][i])) for k in RAW}
        assert abs(ctrl.step(s, float(vp[i]))['v_cmd'] - out['v_cmd'][i]) < 2e-3


def test_batch_controller_never_exceeds_plan_and_respects_stopping_room(ctrl):
    st, vp = random_states(300, seed=1)
    for use in (True, False):
        out = batch_step(ctrl, st, vp, np.full(300, use))
        assert (out['v_cmd'] <= vp + 1e-9).all() and (out['v_cmd'] >= 0).all()
        guard = (st['lease_status'] != 1) & (st['peer_intent'] == 1)
        room = np.maximum(st['d_ego'] - st['d_safe'], 0)
        limit = np.sqrt(2 * st['a_max'] * room)
        assert (out['v_cmd'][guard] <= limit[guard] + 1e-9).all()


def test_same_seed_gives_same_world():
    p = Params(tasks_per_robot=2)
    a, b = Scenario(5, 6, p), Scenario(5, 6, p)
    assert (a.route == b.route).all() and (a.stall == b.stall).all() and (a.delay_ticks == b.delay_ticks).all()
    c = Scenario(6, 6, p)
    assert not (a.route.shape == c.route.shape and (a.route == c.route).all())


def test_routes_have_no_u_turns_and_use_real_lanes():
    p = Params(tasks_per_robot=4)
    sc = Scenario(3, 8, p)
    for i in range(sc.n):
        r = sc.route[i, :sc.route_len[i]]
        assert all(r[k] != r[k + 2] for k in range(len(r) - 2))
        assert (sc.lane_id[i, :sc.route_len[i] - 1] >= 0).all()


def test_single_robot_is_identical_under_both_policies(ctrl):
    """With no other robot there is nothing to predict, so both policies must drive the same."""
    p = Params(tasks_per_robot=3)
    sc = Scenario(2, 1, p)
    a, b = run(sc, 'baseline', ctrl, p), run(sc, 'predictive', ctrl, p)
    assert a['finished'] and b['finished']
    assert a['mean_task_time'] == pytest.approx(b['mean_task_time'])


@pytest.mark.parametrize('policy', ['baseline', 'predictive'])
def test_small_fleet_finishes_without_collisions(ctrl, policy):
    p = Params(tasks_per_robot=2)
    for seed in (1, 2):
        r = run(Scenario(seed, 6, p), policy, ctrl, p)
        assert r['finished'] and not r['deadlock']
        assert r['collisions'] == 0 and r['lease_violations'] == 0


def test_crossing_robots_are_not_flagged_as_rear_end(ctrl):
    """Regression: a robot crossing in from another lane used to look like a rear-end overlap."""
    p = Params(tasks_per_robot=5)
    r = run(Scenario(1, 10, p), 'predictive', ctrl, p)
    assert r['rear_end'] == 0 and r['zone_collisions'] == 0


def test_lease_head_of_lane_rule_prevents_the_queue_deadlock(ctrl):
    """Regression: a follower used to get the lease while its leader waited, and both stuck."""
    p = Params(tasks_per_robot=5)
    r = run(Scenario(1, 10, p), 'baseline', ctrl, p)
    assert r['finished'] and not r['deadlock']


def test_safety_check_actually_detects_a_violation(ctrl):
    """The collision counter must be able to fire: a robot that ignores its stopping limit is caught."""
    import traffic.control as tc

    original = tc.batch_step

    def reckless(ctrl_, st, v_plan, use_model, *a, **k):
        out = original(ctrl_, st, v_plan, use_model, *a, **k)
        out['v_cmd'] = np.asarray(v_plan, dtype=float)      # ignore the safety layer entirely
        return out

    tc.batch_step = reckless
    try:
        p = Params(tasks_per_robot=4, nx=3, ny=3)
        bad = [run(Scenario(s, 12, p), 'baseline', ctrl, p) for s in (1, 2)]
    finally:
        tc.batch_step = original
    assert sum(r['collisions'] + r['lease_violations'] for r in bad) > 0

"""The safety layer is what guarantees stopping room, so it gets its own tests."""
import math
import random
import sys
import warnings
from pathlib import Path

import pytest

pytest.importorskip('lightgbm')
warnings.filterwarnings('ignore')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from speed_controller import SpeedController   # noqa: E402

BASE = dict(v_ego=1.0, d_ego=4.0, v_peer=0.8, d_peer=2.0, peer_intent=1, lease_status=0,
            priority_diff=0, payload_kg=10, msg_age_ms=60, a_max=0.9, robot_len=0.8, d_safe=0.3,
            loc_age_s=0.5, heartbeat_ok=1, blocked_edge=0)


@pytest.fixture(scope='module')
def ctrl():
    return SpeedController()


def test_blocked_edge_stops(ctrl):
    out = ctrl.step(dict(BASE, blocked_edge=1), v_plan=1.0)
    assert out['v_cmd'] == 0 and out['action_state'] == 'STOP'


def test_stale_peer_data_falls_back_to_crawl(ctrl):
    out = ctrl.step(dict(BASE, msg_age_ms=1800), v_plan=1.0)
    assert out['fallback_flag'] == 1 and out['v_cmd'] <= 0.1


def test_lost_heartbeat_and_old_localization_fall_back(ctrl):
    assert ctrl.step(dict(BASE, heartbeat_ok=0), v_plan=1.0)['fallback_flag'] == 1
    assert ctrl.step(dict(BASE, loc_age_s=9.0), v_plan=1.0)['fallback_flag'] == 1


def test_denied_lease_close_to_junction_stops(ctrl):
    out = ctrl.step(dict(BASE, lease_status=2, d_ego=0.8, v_ego=0.5), v_plan=1.0)
    assert out['v_cmd'] == 0


def test_no_conflicting_peer_keeps_the_planned_speed(ctrl):
    out = ctrl.step(dict(BASE, peer_intent=0), v_plan=0.9)
    assert out['v_cmd'] == pytest.approx(0.9)


def test_never_faster_than_plan_or_stopping_limit(ctrl):
    """Random states: the command must never exceed the plan, and without a lease
    it must never exceed the speed that still lets the robot stop before the junction."""
    rng = random.Random(7)
    for _ in range(300):
        s = dict(BASE,
                 v_ego=rng.uniform(0.1, 1.0), d_ego=rng.uniform(0.5, 8.0),
                 v_peer=rng.uniform(0.0, 1.0), d_peer=rng.uniform(-1.0, 8.0),
                 peer_intent=rng.choice([0, 1]), lease_status=rng.choice([0, 1, 2]),
                 payload_kg=rng.choice([0, 10, 30]), msg_age_ms=rng.uniform(0, 2500),
                 a_max=rng.uniform(0.6, 1.1), loc_age_s=rng.uniform(0, 8),
                 heartbeat_ok=rng.choice([0, 1]), blocked_edge=int(rng.random() < 0.1))
        v_plan = rng.uniform(0.3, 1.0)
        out = ctrl.step(s, v_plan)
        assert 0 <= out['v_cmd'] <= v_plan + 1e-3
        if s['peer_intent'] == 1 and s['lease_status'] != 1:
            room = s['d_ego'] - s['d_safe']
            limit = math.sqrt(2 * s['a_max'] * room) if room > 0 else 0.0
            assert out['v_cmd'] <= limit + 1e-3

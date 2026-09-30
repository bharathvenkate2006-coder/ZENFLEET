"""
Bridge between the CNP bidding code and the ZenFleet prediction models.

Two things come out of here:

1. Energy and range.  A robot's battery percentage becomes "how many metres can I
   still drive with this payload?", so a robot with a full battery but a heavy load
   does not over-promise.  This part is a physics formula (constants live in the
   config file), not a trained model.

2. Junction delay.  For every shared junction on a candidate route we ask the
   trained ZenFleet models (Q50/Q90 clearing time and conflict probability) how long
   this robot would probably have to wait for another robot that is heading through
   the same junction.  That expected wait goes into the bid, so a robot whose route
   runs into traffic bids higher than one with a clear road.

If the ZenFleet models cannot be loaded (for example LightGBM is not installed) the
adapter falls back to a simple physics estimate and says so in every bid
(`prediction_source`), so nobody mistakes one for the other.
"""
import math
import sys
import warnings
from functools import lru_cache
from pathlib import Path

# cnp/bidding/predictors.py  ->  parents[2] is the ZenFleet repository root
ZENFLEET_ROOT = Path(__file__).resolve().parents[2]
EPS = 0.1
T_CAP = 40.0


@lru_cache(maxsize=4)
def _load_controller(root):
    """Load the ZenFleet SpeedController once. Returns (controller, error_text)."""
    src = str(Path(root) / 'src')
    sys.path.insert(0, src)
    try:
        from speed_controller import SpeedController
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            return SpeedController(), None
    except Exception as exc:  # missing lightgbm, missing model files, ...
        return None, f'{type(exc).__name__}: {exc}'
    finally:
        if src in sys.path:
            sys.path.remove(src)


_ADAPTERS = {}


def make_predictor(config):
    """Return a shared PredictionAdapter, or None when the config does not ask for one."""
    if not config.get('predictor', {}).get('enabled'):
        return None
    import json
    key = json.dumps({k: config.get(k) for k in ('predictor', 'energy', 'junctions', 'graph')},
                     sort_keys=True)
    if key not in _ADAPTERS:
        _ADAPTERS[key] = PredictionAdapter(config)
    return _ADAPTERS[key]


class PredictionAdapter:
    def __init__(self, config):
        self.graph = config['graph']
        self.junctions = set(config.get('junctions', []))
        self.energy = config.get('energy') or {}
        self.heartbeat_timeout = config.get('heartbeat_timeout', 0.8)
        pc = config.get('predictor', {})
        self.use_congestion = pc.get('use_congestion', True)
        self.buffer_s = pc.get('buffer_s', 0.5)
        self.robot_len = pc.get('robot_len', 0.8)
        self.d_safe = pc.get('d_safe', 0.3)
        self.ctrl, self.model_error = None, 'models disabled in config'
        if pc.get('use_models', True):
            root = pc.get('zenfleet_root') or ZENFLEET_ROOT
            self.ctrl, self.model_error = _load_controller(str(root))
        self.source = 'lightgbm' if self.ctrl is not None else 'physics'

    # ------------------------------------------------------------------ energy
    @property
    def has_energy(self):
        return bool(self.energy)

    def speed(self, robot):
        return max(robot.get('cruise_speed', self.energy.get('cruise_speed', 0.8)), EPS)

    def mass(self, robot, payload):
        return robot.get('mass_kg', self.energy.get('robot_mass_kg', 15.0)) + payload

    def energy_per_m(self, robot, payload):
        """Wh per metre = baseline + weight term + speed-squared term."""
        e, v = self.energy, self.speed(robot)
        return e.get('c0', 0.06) + e.get('c1', 0.004) * self.mass(robot, payload) + e.get('c2', 0.03) * v * v

    def range_m(self, robot, payload):
        e = self.energy
        usable_pct = max(0.0, robot['battery'] - e.get('reserve_percent', 10.0))
        return usable_pct / 100.0 * e.get('capacity_wh', 100.0) / self.energy_per_m(robot, payload)

    def a_max(self, robot, payload):
        empty = self.energy.get('a_max_empty', 1.0)
        base = robot.get('mass_kg', self.energy.get('robot_mass_kg', 15.0))
        return empty * (base / (base + payload)) ** 0.5

    # ------------------------------------------------------- junction prediction
    def _cumulative(self, route):
        out = [0.0]
        for a, b in zip(route, route[1:]):
            out.append(out[-1] + self.graph.get(a, {}).get(b, 0.0))
        return out

    def junction_delay(self, robot_id, robot, route, peers, now, payload):
        """Expected seconds lost at shared junctions on `route`, from other robots' broadcast intent."""
        cum = self._cumulative(route)
        v = self.speed(robot)
        states, meta = [], []
        for idx, node in enumerate(route):
            if idx == 0 or node not in self.junctions:
                continue
            for pid, info in sorted(peers.items()):
                if pid == robot_id or now - info['seen'] > self.heartbeat_timeout:
                    continue
                peer_route = info.get('route') or []
                if node not in peer_route:
                    continue
                d_peer = self._cumulative(peer_route)[peer_route.index(node)]
                age_ms = (now - info['seen']) * 1000
                states.append(dict(
                    v_ego=v, d_ego=cum[idx], v_peer=float(info.get('speed', 0.0)), d_peer=d_peer,
                    peer_intent=1, lease_status=0,
                    priority_diff=int(robot.get('priority', 0) - info.get('priority', 0)),
                    payload_kg=float(payload), msg_age_ms=age_ms, a_max=self.a_max(robot, payload),
                    robot_len=self.robot_len, d_safe=self.d_safe,
                    loc_age_s=float(robot.get('loc_age_s', 0.5)),
                    heartbeat_ok=int(age_ms < 1500), blocked_edge=0))
                meta.append((node, pid))
        details, delay, worst = [], 0.0, 0.0
        if states:
            for (node, pid), s, (t50, t90, risk, t_ego) in zip(meta, states, self._predict(states)):
                wait = max(0.0, t90 + self.buffer_s - t_ego)
                delay += risk * wait
                worst = max(worst, risk)
                details.append(dict(junction=node, peer=pid, conflict_risk=round(risk, 3),
                                    t_clear_q50=round(t50, 2), t_clear_q90=round(t90, 2),
                                    wait_s=round(wait, 2)))
        return dict(expected_delay_s=round(delay, 3), max_risk=round(worst, 3),
                    junctions=details, source=self.source)

    def _predict(self, states):
        """Yield (q50, q90, conflict_risk, t_ego_kin) for each state."""
        if self.ctrl is None:
            return [self._physics(s) for s in states]
        import pandas as pd
        ctrl = self.ctrl
        rows = [ctrl.add_derived(s) for s in states]
        X = pd.DataFrame([[r[f] for f in ctrl.feats] for r in rows], columns=ctrl.feats)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            r50, r90 = ctrl.m50.predict(X), ctrl.m90.predict(X)
            risk = ctrl.clf.predict_proba(X)[:, 1]
        out = []
        for row, a, b, p in zip(rows, r50, r90, risk):
            t50 = row['t_peer_clear_kin'] + float(a)
            t90 = max(row['t_peer_clear_kin'] + float(b), t50)
            out.append((t50, t90, float(p), row['t_ego_kin']))
        return out

    def _physics(self, s):
        """Fallback when the trained models are unavailable: plain kinematics."""
        jl = 1.0 + s['robot_len']
        v_ego, v_peer = max(s['v_ego'], EPS), max(s['v_peer'], EPS)
        t_ego = min(s['d_ego'] / v_ego, T_CAP)
        t_peer_enter = max(s['d_peer'], 0) / v_peer
        t_peer_clear = min((max(s['d_peer'], 0) + jl) / v_peer, T_CAP)
        t_ego_clear = t_ego + jl / v_ego
        risk = 1.0 if (t_ego < t_peer_clear + self.buffer_s and t_peer_enter < t_ego_clear) else 0.0
        return t_peer_clear, min(t_peer_clear * 1.3, T_CAP), risk, t_ego

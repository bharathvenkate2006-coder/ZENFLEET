"""Vectorised copy of the ZenFleet speed controller, so a whole fleet is handled in one call.

`SpeedController.step` (src/speed_controller.py) works on one robot at a time and takes about
3 ms per call. A 20-robot run needs it thousands of times, so this module applies the same rules
to arrays of robots. tests/test_traffic.py checks that both give the same answer on random states.

Two policies share everything except step 3 (the speed suggestion):

  baseline    suggests the planned speed. The safety layer then makes the robot brake as late as
              it safely can and wait at the stop line if the junction is not yet its own.
  predictive  suggests distance / (Q90 clearing time + buffer) x (1 - conflict risk), using the
              trained models, for rows where `use_model` is True.

The safety layer (step 1 and step 4 of the controller: stale data, stopping distance, denied
lease) is identical for both.
"""
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import speed_controller as sc  # noqa: E402

RAW = ["v_ego", "d_ego", "v_peer", "d_peer", "peer_intent", "lease_status", "priority_diff",
       "payload_kg", "msg_age_ms", "a_max", "robot_len", "d_safe", "loc_age_s", "heartbeat_ok",
       "blocked_edge"]


def load_controller():
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return sc.SpeedController()


def batch_step(ctrl, st, v_plan, use_model, buffer_s=None, gamma=None):
    """Same rules as SpeedController.step, for many robots at once.

    st         dict of numpy arrays, one entry per raw feature in RAW (all the same length)
    v_plan     array of planned speeds
    use_model  bool array; rows where the trained models are consulted (others suggest v_plan)
    Returns dict(v_cmd, fallback, r_hat, model_used).
    """
    buffer_s = sc.BUFFER_S if buffer_s is None else buffer_s
    gamma = sc.GAMMA if gamma is None else gamma
    g = {k: np.asarray(st[k], dtype=float) for k in RAW}
    n = len(v_plan)
    v_plan = np.asarray(v_plan, dtype=float)
    use_model = np.asarray(use_model, dtype=bool)
    v_ego, d_ego, v_peer, d_peer = g["v_ego"], g["d_ego"], g["v_peer"], g["d_peer"]

    d_stop = v_ego ** 2 / (2 * g["a_max"])
    t_ego_kin = np.minimum(d_ego / np.maximum(v_ego, sc.EPS), 40.0)
    kin = np.minimum((np.maximum(d_peer, 0) + sc.JUNCTION_LEN + g["robot_len"])
                     / np.maximum(v_peer, sc.EPS), 40.0)
    derived = dict(d_stop=d_stop, stop_margin=d_ego - d_stop, t_ego_kin=t_ego_kin,
                   t_peer_clear_kin=kin, t_gap=t_ego_kin - kin)

    stop = g["blocked_edge"] != 0
    fallback = (g["msg_age_ms"] > sc.STALE_MS) | (g["heartbeat_ok"] == 0) | (g["loc_age_s"] > sc.LOC_STALE_S)

    r_hat = np.zeros(n)
    t90 = np.zeros(n)
    idx = np.where(use_model)[0]
    if idx.size:
        cols = {f: (g[f] if f in g else derived[f])[idx] for f in ctrl.feats}
        X = pd.DataFrame(cols, columns=ctrl.feats)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            r50 = ctrl.m50.predict(X)
            r90 = ctrl.m90.predict(X)
            p = ctrl.clf.predict_proba(X)[:, 1]
        t50m = kin[idx] + r50
        t90m = np.maximum(kin[idx] + r90, t50m)
        r_hat[idx] = p
        t90[idx] = t90m
        fallback[idx] |= (t90m - t50m) > sc.MAX_UNCERT_S

    intent = g["peer_intent"] == 1
    t_target = t90 + buffer_s
    v_target = d_ego / np.maximum(t_target, sc.EPS)
    v_model = v_target * (1 - r_hat) ** gamma
    suggestion = np.where(use_model, v_model, v_plan)
    v_cmd = np.where(~intent, v_plan, np.where(fallback, sc.V_MIN, suggestion))
    v_cmd = np.minimum(v_cmd, v_plan)

    lease = g["lease_status"]
    guard = (lease != sc.LEASE_GRANTED) & intent
    room = d_ego - g["d_safe"]
    limit = np.where(room > 0, np.sqrt(2 * g["a_max"] * np.maximum(room, 0)), 0.0)
    v_cmd = np.where(guard, np.minimum(v_cmd, limit), v_cmd)
    denied_near = guard & (lease == sc.LEASE_DENIED) & (d_ego < d_stop + 2 * g["d_safe"])
    v_cmd = np.where(denied_near, 0.0, v_cmd)
    v_cmd = np.where(stop | ((v_cmd < sc.V_MIN) & guard), 0.0, v_cmd)
    return dict(v_cmd=np.maximum(v_cmd, 0.0), fallback=fallback, r_hat=r_hat, model_used=use_model)

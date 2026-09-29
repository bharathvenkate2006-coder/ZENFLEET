"""
Speed controller = trained models (advisory) + deterministic safety layer (final say).

The model can only LOWER speed or match the plan. It can never raise speed above
what the safety layer allows.

Usage:
    ctrl = SpeedController("models")
    out = ctrl.step(state, v_plan=1.0)
"""
import json, math
from pathlib import Path
import joblib
import numpy as np
import pandas as pd

# ---- tunable parameters -----------------------------------------------------
BUFFER_S      = 0.5     # extra time after peer clears
GAMMA         = 1.0     # risk aggressiveness
V_MIN         = 0.10    # m/s, crawl speed (below this we command a full stop)
STALE_MS      = 1000    # peer data older than this => not trusted
LOC_STALE_S   = 5.0     # time since last ArUco fix => not trusted
MAX_UNCERT_S  = 8.0     # q90 - q50 above this => model unsure
EPS           = 0.1
JUNCTION_LEN  = 1.0

LEASE_PENDING, LEASE_GRANTED, LEASE_DENIED = 0, 1, 2


class SpeedController:
    def __init__(self, model_dir=None, meta_path=None):
        root = Path(__file__).resolve().parents[1]
        model_dir = Path(model_dir) if model_dir else root / "models"
        meta_path = Path(meta_path) if meta_path else root / "config" / "meta.json"
        self.m50 = joblib.load(model_dir / "ttc_q50.pkl")
        self.m90 = joblib.load(model_dir / "ttc_q90.pkl")
        self.clf = joblib.load(model_dir / "conflict_clf.pkl")
        meta = json.load(open(meta_path))
        self.feats = meta["features"]
        self.threshold = meta["conflict_threshold"]

    # ---- derived features (identical to the dataset generator) --------------
    @staticmethod
    def add_derived(s):
        s = dict(s)
        s["d_stop"] = s["v_ego"] ** 2 / (2 * s["a_max"])
        s["stop_margin"] = s["d_ego"] - s["d_stop"]
        s["t_ego_kin"] = min(s["d_ego"] / max(s["v_ego"], EPS), 40.0)
        s["t_peer_clear_kin"] = min(
            (max(s["d_peer"], 0) + JUNCTION_LEN + s["robot_len"]) / max(s["v_peer"], EPS), 40.0)
        s["t_gap"] = s["t_ego_kin"] - s["t_peer_clear_kin"]
        return s

    def step(self, state, v_plan):
        s = self.add_derived(state)
        reasons = []

        # ---------- 1) hard safety conditions (model is NOT consulted) --------
        stop = False
        if s["blocked_edge"]:
            stop = True; reasons.append("blocked_edge")
        fallback = (s["msg_age_ms"] > STALE_MS or not s["heartbeat_ok"]
                    or s["loc_age_s"] > LOC_STALE_S)
        if fallback:
            reasons.append("stale_or_lost_data")

        # ---------- 2) model predictions (advisory) ---------------------------
        x = pd.DataFrame([[s[f] for f in self.feats]], columns=self.feats)
        kin = s["t_peer_clear_kin"]
        t50 = kin + float(self.m50.predict(x)[0])
        t90 = max(kin + float(self.m90.predict(x)[0]), t50)
        r_hat = float(self.clf.predict_proba(x)[0, 1])
        uncertainty = t90 - t50
        if uncertainty > MAX_UNCERT_S:
            fallback = True; reasons.append("model_uncertain")

        # ---------- 3) speed suggestion ---------------------------------------
        if s["peer_intent"] == 0:
            v_cmd = v_plan                                   # no conflict expected
        elif fallback:
            v_cmd = V_MIN                                    # conservative
        else:
            t_target = t90 + BUFFER_S
            v_target = s["d_ego"] / max(t_target, EPS)
            v_cmd = v_target * (1 - r_hat) ** GAMMA
        v_cmd = min(v_cmd, v_plan)                           # never exceed the plan

        # ---------- 4) safety layer: can I still stop before the junction? ----
        if s["lease_status"] != LEASE_GRANTED and s["peer_intent"] == 1:
            room = s["d_ego"] - s["d_safe"]
            v_stop_limit = math.sqrt(2 * s["a_max"] * room) if room > 0 else 0.0
            if v_cmd > v_stop_limit:
                reasons.append("stop_distance_limit")
            v_cmd = min(v_cmd, v_stop_limit)
            if s["lease_status"] == LEASE_DENIED and s["d_ego"] < s["d_stop"] + 2 * s["d_safe"]:
                v_cmd = 0.0; reasons.append("lease_denied_near_junction")

        if stop or v_cmd < V_MIN and s["peer_intent"] == 1 and s["lease_status"] != LEASE_GRANTED:
            v_cmd = 0.0

        v_cmd = max(0.0, v_cmd)
        alpha = v_cmd / v_plan if v_plan > 0 else 0.0
        if v_cmd == 0:            action = "STOP"
        elif alpha < 0.5:         action = "YIELD"
        elif alpha < 0.95:        action = "SLOW"
        else:                     action = "GO"

        return dict(v_cmd=round(v_cmd, 3), alpha=round(alpha, 3), action_state=action,
                    fallback_flag=int(fallback), r_hat=round(r_hat, 3),
                    t_clear_peer_q50=round(t50, 2), t_clear_peer_q90=round(t90, 2),
                    uncertainty_s=round(uncertainty, 2), reasons=reasons)

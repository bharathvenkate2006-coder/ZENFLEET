"""
Synthetic dataset generator: two-robot junction crossing (SIH26123 MVP).

Replace ROBOT dict values with the specs of YOUR simulated AMR.
Later, replace this generator with logs from your real simulator (Gazebo /
Isaac Sim) using the same column names, so the training code does not change.

Usage:  python generate_amr_dataset.py --n 60000 --seed 42
"""
import argparse
import numpy as np
import pandas as pd

# ---- ASSUMED robot parameters (edit to match your AMR) ---------------------
ROBOT = dict(
    v_max=1.0,          # m/s   (example Open-AMR-class bot is 0.5 m/s)
    length=0.8,         # m
    robot_mass=15.0,    # kg
    a_max_empty=1.0,    # m/s^2 braking capability with no payload
    d_safe=0.3,         # m     safety margin
    junction_len=1.0,   # m     length of the shared junction zone
)
DT = 0.05
T_MAX = 40.0


def simulate_position_profile(v0, d0, n, rng, v_max, stall_p):
    """Simulate a robot approaching and crossing the junction.
    Returns enter_time and clear_time (seconds from now) for each sample."""
    steps = int(T_MAX / DT)
    pos = -d0.copy()                     # junction entry is at pos = 0
    v = v0.copy()
    target = v0.copy()
    stalled_until = np.zeros(n)
    t_enter = np.full(n, np.nan)
    t_clear = np.full(n, np.nan)
    end = ROBOT["junction_len"] + ROBOT["length"]
    for k in range(steps):
        t = k * DT
        if k % 10 == 0:  # every 0.5 s the robot changes its speed target a bit
            target = np.clip(target + rng.normal(0, 0.08, n), 0.15, v_max)
            new_stall = rng.random(n) < stall_p
            stalled_until = np.where(new_stall, t + rng.uniform(0.5, 3.0, n), stalled_until)
        stalled = t < stalled_until
        goal = np.where(stalled, 0.0, target)
        v += np.clip(goal - v, -1.0 * DT, 0.8 * DT)      # accel / decel limits
        pos += v * DT
        t_enter = np.where(np.isnan(t_enter) & (pos >= 0), t, t_enter)
        t_clear = np.where(np.isnan(t_clear) & (pos >= end), t, t_clear)
        if not np.isnan(t_clear).any():
            break
    t_clear = np.where(np.isnan(t_clear), T_MAX, t_clear)
    t_enter = np.where(np.isnan(t_enter), T_MAX, t_enter)
    return t_enter, t_clear


def generate(n, seed):
    rng = np.random.default_rng(seed)
    R = ROBOT

    layout_id = rng.integers(0, 10, n)
    episode_id = layout_id * 100000 + rng.integers(0, 2000, n)

    payload = rng.choice([0, 5, 10, 20, 30], n, p=[.3, .2, .2, .2, .1]).astype(float)
    a_max = R["a_max_empty"] * (R["robot_mass"] / (R["robot_mass"] + payload)) ** 0.5
    a_max *= rng.uniform(0.9, 1.1, n)      # friction / wear variation

    v_ego = rng.uniform(0.1, R["v_max"], n)
    d_ego = rng.uniform(0.5, 8.0, n)
    v_peer = rng.uniform(0.0, R["v_max"], n)
    d_peer = rng.uniform(-1.0, 8.0, n)     # negative = peer already inside junction
    peer_intent = (rng.random(n) < 0.85).astype(int)

    priority_diff = rng.integers(-3, 4, n)
    lease_status = rng.choice([0, 1, 2], n, p=[.4, .4, .2])   # 0 pending,1 granted,2 denied
    msg_age_ms = np.clip(rng.gamma(2.0, 40.0, n), 0, 2000)
    dropout = rng.random(n) < 0.03
    msg_age_ms = np.where(dropout, rng.uniform(1000, 3000, n), msg_age_ms)
    loc_age_s = np.clip(rng.exponential(1.5, n), 0, 20)
    heartbeat_ok = (msg_age_ms < 1500).astype(int)
    blocked_edge = (rng.random(n) < 0.02).astype(int)

    # ---- ground truth (what really happens if nobody intervenes) ----------
    stall_p = 0.03
    pe_enter, pe_clear = simulate_position_profile(v_peer.clip(0.05), d_peer, n, rng, R["v_max"], stall_p)
    eg_enter, eg_clear = simulate_position_profile(v_ego, d_ego, n, rng, R["v_max"], stall_p)
    pe_enter = np.where(d_peer < 0, 0.0, pe_enter)

    overlap = (eg_enter < pe_clear) & (pe_enter < eg_clear) & (peer_intent == 1)

    # ---- what the robot actually OBSERVES (stale + noisy) ------------------
    age_s = msg_age_ms / 1000
    v_peer_obs = np.clip(v_peer + rng.normal(0, 0.03, n), 0, None)
    d_peer_obs = d_peer + v_peer * age_s + rng.normal(0, 0.05, n)  # peer moved since message
    v_ego_obs = np.clip(v_ego + rng.normal(0, 0.02, n), 0, None)
    d_ego_obs = d_ego + rng.normal(0, 0.03 + 0.02 * loc_age_s, n)

    # ---- derived (mandatory) features --------------------------------------
    eps = 0.1
    L, Lj = R["length"], R["junction_len"]
    d_stop = v_ego_obs ** 2 / (2 * a_max)
    t_ego_kin = np.minimum(d_ego_obs / np.maximum(v_ego_obs, eps), T_MAX)
    t_peer_clear_kin = np.minimum((np.maximum(d_peer_obs, 0) + Lj + L) / np.maximum(v_peer_obs, eps), T_MAX)
    t_gap = t_ego_kin - t_peer_clear_kin

    df = pd.DataFrame(dict(
        episode_id=episode_id, layout_id=layout_id,
        v_ego=v_ego_obs, d_ego=d_ego_obs, v_peer=v_peer_obs, d_peer=d_peer_obs,
        peer_intent=peer_intent, lease_status=lease_status, priority_diff=priority_diff,
        payload_kg=payload, msg_age_ms=msg_age_ms, a_max=a_max,
        robot_len=L, d_safe=R["d_safe"], loc_age_s=loc_age_s,
        heartbeat_ok=heartbeat_ok, blocked_edge=blocked_edge,
        d_stop=d_stop, stop_margin=d_ego_obs - d_stop,
        t_ego_kin=t_ego_kin, t_peer_clear_kin=t_peer_clear_kin, t_gap=t_gap,
        # ---- labels ----
        y_t_clear_peer=pe_clear,                       # regression target (s)
        y_residual=pe_clear - t_peer_clear_kin,        # residual over physics baseline
        y_t_enter_ego=eg_enter, y_t_clear_ego=eg_clear,
        y_conflict=overlap.astype(int),                # classification target
        y_min_sep_violation=(overlap & (np.abs(eg_enter - pe_clear) < 0.5)).astype(int),
    ))
    # split by layout so the test set is a layout never seen in training
    df["split"] = np.where(df.layout_id == 9, "test",
                  np.where(df.layout_id == 8, "val", "train"))
    return df.round(4)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="amr_junction_dataset.csv")
    a = ap.parse_args()
    df = generate(a.n, a.seed)
    df.to_csv(a.out, index=False)
    print(df.shape)
    print(df.split.value_counts().to_dict())
    print("conflict rate:", df.y_conflict.mean().round(3))

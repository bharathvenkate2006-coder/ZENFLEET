"""Multi-robot warehouse traffic simulation (time step 0.1 s).

What is simulated, in one paragraph: robots drive one-way along the two lanes of every aisle of a
grid map and cross shared junction zones. Each junction is guarded by an exclusive lease. Every
robot broadcasts its position and speed at 10 Hz over a link with delay and random outages.
Each tick a robot picks a speed from the policy under test, and physics (acceleration limits,
payload-dependent braking, random stalls, sensor noise) moves it. Collisions are measured from the
true positions, not from what the robots believe.

Fair comparison: for a given seed, both policies get the same map, tasks, robots, stalls, message
delays and outages, and sensor noise (all drawn in advance, independent of the policy).

What is NOT simulated: the lease protocol itself is an abstract arbiter with message latency (the
quorum lease lives in cnp/), turns cost no time, and there is no robot dynamics beyond speed limits.
"""
from dataclasses import dataclass

import numpy as np

from . import control as ctl

LEASE_PENDING, LEASE_GRANTED, LEASE_DENIED = 0, 1, 2


@dataclass(frozen=True)
class Params:
    # map
    nx: int = 5
    ny: int = 4
    aisle: float = 4.0          # m between node centres
    junction: float = 1.0       # m, length of the shared junction zone
    robot_len: float = 0.8
    d_safe: float = 0.3
    marker_spacing: float = 2.0  # floor markers, one position fix every 2 m
    # workload
    tasks_per_robot: int = 5
    t_max: float = 600.0
    dt: float = 0.1
    # robot physics (same assumptions as src/generate_dataset.py)
    a_up: float = 0.8
    a_empty: float = 1.0
    robot_mass: float = 15.0
    stall_p: float = 0.03        # chance per 0.5 s that a robot stalls
    # communication
    delay_shape: float = 2.0
    delay_scale: float = 0.04    # gamma(2, 40 ms) like the training data
    outage_p: float = 0.0015     # per link per tick; 1-3 s long
    # leases
    grant_dist: float = 2.0      # a robot asks for the lease this close to the junction
    lane_cap: int = 2
    # control
    ctrl_dist: float = 8.0       # models were trained for junctions up to 8 m away
    follow_gap: float = 0.5
    follow_decel: float = 1.1
    gate_stationary: bool = True  # only consult the models for a moving predecessor
    moving_speed: float = 0.15
    buffer_s: float = None
    gamma: float = None


def grid_lanes(p):
    nodes = [(x, y) for y in range(p.ny) for x in range(p.nx)]
    nid = {xy: i for i, xy in enumerate(nodes)}
    nbrs = {i: [] for i in range(len(nodes))}
    lanes = {}
    for (x, y), i in nid.items():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            j = nid.get((x + dx, y + dy))
            if j is not None:
                nbrs[i].append(j)
                lanes[(i, j)] = len(lanes)
    return nodes, nbrs, lanes


def _route(rng, nodes, nbrs, start, prev, cur, m_tasks):
    """Concatenated route through m_tasks destinations; no U-turns."""
    route, tend = [start, cur] if prev is None else [prev, cur], []
    prev, cur = route[-2], route[-1]
    for _ in range(m_tasks):
        while True:
            dest = int(rng.integers(len(nodes)))
            far = abs(nodes[dest][0] - nodes[cur][0]) + abs(nodes[dest][1] - nodes[cur][1])
            if far >= 2:
                break
        frontier, seen = [(cur, prev, [])], {(cur, prev)}
        path = None
        while frontier and path is None:
            nxt = []
            for node, pv, pth in frontier:
                for nb in nbrs[node]:
                    if nb == pv or (nb, node) in seen:
                        continue
                    seen.add((nb, node))
                    q = pth + [nb]
                    if nb == dest:
                        path = q
                        break
                    nxt.append((nb, node, q))
                if path:
                    break
            frontier = nxt
        route.extend(path)
        tend.append(len(route) - 1)
        prev, cur = route[-2], route[-1]
    return route, tend


class Scenario:
    """Everything random, drawn once per (seed, fleet size) so both policies see the same world."""

    def __init__(self, seed, n, p):
        self.p, self.n, self.seed = p, n, seed
        rng = np.random.default_rng([seed, n])
        nodes, nbrs, lanes = grid_lanes(p)
        self.nodes, self.lanes, self.nlanes = nodes, lanes, len(lanes)
        T = int(p.t_max / p.dt)
        self.T = T
        # routes
        keys = list(lanes.keys())
        pick = rng.choice(len(keys), size=n, replace=False)
        routes, tends = [], []
        for i in range(n):
            u, v = keys[pick[i]]
            r, te = _route(rng, nodes, nbrs, u, u, v, p.tasks_per_robot)
            routes.append(r)
            tends.append(te)
        K = max(len(r) for r in routes)
        self.route = np.full((n, K), -1, dtype=int)
        self.lane_id = np.full((n, K), -1, dtype=int)
        for i, r in enumerate(routes):
            self.route[i, :len(r)] = r
            for k in range(len(r) - 1):
                self.lane_id[i, k] = lanes[(r[k], r[k + 1])]
        self.tend = np.array(tends, dtype=int)
        self.route_len = np.array([len(r) for r in routes])
        # robots
        payload = rng.choice([0, 5, 10, 20, 30], n, p=[.3, .2, .2, .2, .1]).astype(float)
        self.payload = payload
        self.a_nom = p.a_empty * (p.robot_mass / (p.robot_mass + payload)) ** 0.5
        self.a_true = self.a_nom * rng.uniform(0.9, 1.1, n)
        self.base_speed = rng.uniform(0.6, 1.0, n)
        # plan speed: random walk every 0.5 s, like the training data
        slots = T // 5 + 2
        drift = rng.normal(0, 0.05, (slots, n))
        tgt = np.empty((slots, n))
        cur = self.base_speed.copy()
        for s in range(slots):
            cur = np.clip(cur + drift[s], 0.4, 1.0)
            tgt[s] = cur
        self.v_plan = np.repeat(tgt, 5, axis=0)[:T]
        # stalls
        starts = rng.random((slots, n)) < p.stall_p
        dur = rng.uniform(0.5, 3.0, (slots, n))
        until = np.zeros(n)
        stall = np.zeros((slots * 5, n), dtype=bool)
        for s in range(slots):
            t0 = s * 0.5
            until = np.where(starts[s], t0 + dur[s], until)
            for k in range(5):
                stall[s * 5 + k] = (t0 + k * p.dt) < until
        self.stall = stall[:T]
        # communication: delay in ticks per link (receiver i, sender j), and outages
        delay = rng.gamma(p.delay_shape, p.delay_scale, (T, n, n))
        self.delay_ticks = np.clip(np.rint(delay / p.dt), 0, 12).astype(np.int8)
        out_start = rng.random((T, n, n)) < p.outage_p
        out_len = rng.integers(10, 31, (T, n, n))
        up = np.ones((T, n, n), dtype=bool)
        down_until = np.zeros((n, n), dtype=int)
        for t in range(T):
            down_until = np.where(out_start[t] & (down_until <= t), t + out_len[t], down_until)
            up[t] = down_until <= t
        self.link_up = up
        # noise
        self.noise = rng.normal(0, 1, (T, n, 6))
        self.lease_lat = rng.gamma(2.0, 0.04, (T, n)) * 2 + (rng.random((T, n)) < 0.03) * 0.3
        self.release_lat = rng.gamma(2.0, 0.04, (T, n))
        # time a robot would need with no other traffic and no stalls (driving at its plan speed)
        s0 = p.junction / 2 + p.robot_len + 0.2
        length = p.aisle * self.tend[:, -1] - s0
        self.free_flow = float(np.mean(length / self.v_plan.mean(axis=0)) / p.tasks_per_robot)


def run(sc, policy, ctrl=None, p=None, trace=None):
    """Run one scenario under 'baseline' or 'predictive'. Returns a dict of metrics."""
    p = p or sc.p
    n, T, dt = sc.n, sc.T, p.dt
    L, J, ln = p.aisle, p.junction, p.robot_len
    ctrl = ctrl or ctl.load_controller()
    use_models = policy == "predictive"
    K = sc.route.shape[1]
    H, KD = 64, 12

    S = np.full(n, J / 2 + ln + 0.2)
    v = np.zeros(n)
    active = np.ones(n, dtype=bool)
    task_i = np.zeros(n, dtype=int)
    task_t0 = np.zeros(n)
    task_times = []
    fix_idx = np.floor(S / p.marker_spacing)
    last_fix = np.zeros(n)
    pos_err = 0.03 * sc.noise[0, :, 5]    # dead-reckoning error: resets at each marker, grows with distance

    nnodes = len(sc.nodes)
    holder = np.full(nnodes, -1)
    free_at = np.zeros(nnodes)
    hold_node = np.full(n, -1)
    hold_k = np.full(n, -1)
    known_at = np.full(n, np.inf)

    hist_S = np.zeros((H, n))
    hist_v = np.zeros((H, n))
    fresh = np.zeros((n, n), dtype=int)
    arrq = np.full((KD + 1, n, n), -1, dtype=int)
    hist_S[0], hist_v[0] = S, v

    wait_ticks = np.zeros(n)
    stall_ticks = np.zeros(n)
    stops = 0
    was_stopped = np.zeros(n, dtype=bool)
    zone_pairs, rear_pairs, prev_zone, prev_rear = set(), set(), set(), set()
    zone_events = rear_events = 0
    violations, viol_seen = 0, set()
    fallback_rows = model_rows = ctrl_rows = 0
    speed_sum = 0.0
    speed_n = 0
    prog = []
    deadlock = False
    idx_n = np.arange(n)
    t_end = T
    fin = False

    for t in range(T):
        now = t * dt
        # ---------------- communication
        hist_S[t % H], hist_v[t % H] = S, v
        for kk in range(KD + 1):
            m = (sc.delay_ticks[t] == kk) & sc.link_up[t] & active[None, :]
            if m.any():
                a = arrq[(t + kk) % (KD + 1)]
                a[m] = np.maximum(a[m], t)
        fresh = np.maximum(fresh, arrq[t % (KD + 1)])
        arrq[t % (KD + 1)] = -1
        # ---------------- geometry
        k_next = np.floor((S - ln - J / 2) / L).astype(int) + 1
        k_next = np.minimum(k_next, K - 1)
        node_next = sc.route[idx_n, k_next]
        d_entry = L * k_next - J / 2 - S
        # ---------------- task completion
        for i in np.where(active & (S >= L * sc.tend[idx_n, np.minimum(task_i, sc.tend.shape[1] - 1)]))[0]:
            task_times.append(now - task_t0[i])
            task_t0[i] = now
            task_i[i] += 1
            if task_i[i] >= sc.tend.shape[1]:
                active[i] = False
                v[i] = 0
                if hold_node[i] >= 0:
                    holder[hold_node[i]] = -1
                    free_at[hold_node[i]] = now + sc.release_lat[t, i]
                    hold_node[i] = -1
        if not active.any():
            fin, t_end = True, t
            break
        # ---------------- leader gaps (local range sensing)
        kf = np.minimum(np.floor(S / L).astype(int), K - 1)
        lf = sc.lane_id[idx_n, kf]
        kt = np.maximum(np.floor((S - ln) / L).astype(int), 0)
        lt = sc.lane_id[idx_n, np.minimum(kt, K - 1)]
        qf, qt = S - L * kf, (S - ln) - L * kt
        both_in_lane = lt == lf
        gap = np.full(n, np.inf)
        gap_body = np.full(n, np.inf)      # only leaders whose whole body is in the lane: real overlaps
        lead = np.full(n, -1)
        lead_body = np.full(n, -1)
        for m in range(3):
            kk = np.minimum(kf + m, K - 1)
            lane_m = np.where(kf + m < K, sc.lane_id[idx_n, kk], -1)
            Dm = L * (kf + m)
            ok = (lane_m >= 0)[:, None] & active[None, :] & active[:, None]
            front_route = Dm[:, None] + qf[None, :]
            ahead = front_route > S[:, None] + 1e-9
            fr = ok & (lf[None, :] == lane_m[:, None]) & ahead
            # a leader that crossed in from another lane still has its tail in the junction zone
            tail1 = np.where(both_in_lane[None, :], front_route - ln, (Dm - J / 2)[:, None])
            g1 = np.where(fr, tail1 - S[:, None], np.inf)
            tail_route = Dm[:, None] + qt[None, :]
            tr = ok & (lt[None, :] == lane_m[:, None]) & (tail_route + ln > S[:, None] + 1e-9)
            g2 = np.where(tr, tail_route - S[:, None], np.inf)
            g = np.minimum(g1, g2)
            g[idx_n, idx_n] = np.inf
            j = g.argmin(axis=1)
            gm = g[idx_n, j]
            better = gm < gap
            gap = np.where(better, gm, gap)
            lead = np.where(better, j, lead)
            gb = np.minimum(np.where(fr & both_in_lane[None, :], front_route - ln - S[:, None], np.inf), g2)
            gb[idx_n, idx_n] = np.inf
            jb = gb.argmin(axis=1)
            gbm = gb[idx_n, jb]
            bb = gbm < gap_body
            gap_body = np.where(bb, gbm, gap_body)
            lead_body = np.where(bb, jb, lead_body)
        gap_obs = gap + 0.02 * sc.noise[t, :, 4]
        v_lead = np.where(lead >= 0, v[np.maximum(lead, 0)], 0.0)
        room_f = np.maximum(gap_obs - p.follow_gap, 0.0)
        v_follow = np.sqrt(2 * sc.a_nom * room_f + v_lead ** 2 * sc.a_nom / p.follow_decel)
        v_follow = np.where(np.isfinite(gap), v_follow, 9.0)
        # ---------------- lease release
        for i in np.where(active & (hold_node >= 0))[0]:
            if S[i] - ln >= L * hold_k[i] + J / 2:
                holder[hold_node[i]] = -1
                free_at[hold_node[i]] = now + sc.release_lat[t, i]
                hold_node[i] = -1
                known_at[i] = np.inf
        # ---------------- lease grant (exclusive, nearest first)
        lane_count = np.bincount(lf[active & (lf >= 0)], minlength=sc.nlanes)
        for i in np.where(active & (hold_node >= 0) & (S < L * np.maximum(hold_k, 0)))[0]:
            lane_count[sc.lane_id[i, hold_k[i]]] += 1
        cand = {}
        for i in np.where(active & (hold_node < 0) & (d_entry <= p.grant_dist) & (node_next >= 0))[0]:
            cand.setdefault(int(node_next[i]), []).append((d_entry[i], i))
        for node, lst in cand.items():
            if holder[node] >= 0 or now < free_at[node]:
                continue
            blocked_in = set()
            for d, i in sorted(lst):
                in_lane = sc.lane_id[i, k_next[i] - 1]
                if in_lane in blocked_in:      # only the robot at the head of its lane may ask
                    continue
                out_lane = sc.lane_id[i, k_next[i]]
                if d > 0 and out_lane >= 0 and lane_count[out_lane] >= p.lane_cap:
                    blocked_in.add(in_lane)
                    continue
                holder[node], hold_node[i], hold_k[i] = i, node, k_next[i]
                known_at[i] = now + sc.lease_lat[t, i]
                if out_lane >= 0:
                    lane_count[out_lane] += 1
                break
        # ---------------- lease status per robot
        status = np.zeros(n, dtype=int)
        mine = active & (hold_node >= 0) & (now >= known_at)
        status[mine] = LEASE_GRANTED
        h_of = np.where(node_next >= 0, holder[np.maximum(node_next, 0)], -1)
        status[active & ~mine & (h_of >= 0) & (h_of != idx_n) & (d_entry <= p.grant_dist)] = LEASE_DENIED
        # ---------------- junction speed control
        v_plan = sc.v_plan[t]
        v_cmd = v_plan.copy()
        rows = np.where(active & (status != LEASE_GRANTED) & (d_entry <= p.ctrl_dist) & (node_next >= 0))[0]
        if rows.size:
            loc_age = now - last_fix
            feats = {k: np.zeros(rows.size) for k in ctl.RAW}
            use_model = np.zeros(rows.size, dtype=bool)
            for r, i in enumerate(rows):
                node = node_next[i]
                fresh_i = fresh[i]
                Sm = hist_S[np.maximum(fresh_i, t - H + 1) % H, idx_n]
                vm = hist_v[np.maximum(fresh_i, t - H + 1) % H, idx_n]
                k0m = np.floor((Sm - ln - J / 2) / L).astype(int) + 1
                best_j, best_d = -1, -np.inf
                for m in range(3):
                    kk = np.minimum(k0m + m, K - 1)
                    hit = (sc.route[idx_n, kk] == node) & active & (idx_n != i)
                    dj = L * (k0m + m) - J / 2 - Sm
                    ahead = hit & ((holder[node] == idx_n) | (dj < d_entry[i] - 0.05)) & (dj <= p.ctrl_dist)
                    if ahead.any():
                        jj = np.where(ahead, dj, -np.inf).argmax()
                        if dj[jj] > best_d:
                            best_j, best_d = int(jj), float(dj[jj])
                nz = sc.noise[t, i]
                feats["v_ego"][r] = max(0.0, v[i] + 0.02 * nz[0])
                feats["d_ego"][r] = d_entry[i] + pos_err[i] + 0.02 * nz[1]
                feats["lease_status"][r] = status[i]
                feats["payload_kg"][r] = sc.payload[i]
                feats["a_max"][r] = sc.a_nom[i]
                feats["robot_len"][r] = ln
                feats["d_safe"][r] = p.d_safe
                feats["loc_age_s"][r] = loc_age[i]
                feats["peer_intent"][r] = 1
                feats["heartbeat_ok"][r] = 1
                feats["d_peer"][r] = 100.0
                if best_j >= 0:
                    age_ms = (t - fresh_i[best_j]) * dt * 1000.0
                    feats["v_peer"][r] = max(0.0, vm[best_j] + 0.03 * sc.noise[t, best_j, 2])
                    feats["d_peer"][r] = best_d + 0.05 * sc.noise[t, best_j, 3]
                    feats["msg_age_ms"][r] = age_ms
                    feats["heartbeat_ok"][r] = int(age_ms < 1500)
                    moving = vm[best_j] >= p.moving_speed or best_d < -0.5
                    use_model[r] = use_models and (moving or not p.gate_stationary)
            out = ctl.batch_step(ctrl, feats, v_plan[rows], use_model, p.buffer_s, p.gamma)
            v_cmd[rows] = out["v_cmd"]
            ctrl_rows += rows.size
            model_rows += int(use_model.sum())
            fallback_rows += int(out["fallback"].sum())
        v_cmd = np.minimum(v_cmd, v_follow)
        v_cmd = np.where(sc.stall[t], 0.0, v_cmd)
        v_cmd = np.where(active, v_cmd, 0.0)
        # ---------------- physics
        v = np.clip(v_cmd, v - sc.a_true * dt, v + p.a_up * dt)
        v = np.where(active, np.maximum(v, 0.0), 0.0)
        S = S + v * dt
        fi = np.floor(S / p.marker_spacing)
        fixed = fi > fix_idx
        last_fix = np.where(fixed, now + dt, last_fix)
        pos_err = np.where(fixed, 0.03 * sc.noise[t, :, 5], pos_err + 0.02 * sc.noise[t, :, 5] * np.sqrt(v * dt))
        fix_idx = fi
        # ---------------- metrics (true positions)
        st_now = active & (v < 0.05) & ~sc.stall[t]
        wait_ticks += st_now
        stall_ticks += active & sc.stall[t]
        stops += int((st_now & ~was_stopped).sum())
        was_stopped = st_now
        speed_sum += v[active].sum()
        speed_n += int(active.sum())
        kz = np.minimum(np.floor((S - ln - J / 2) / L).astype(int) + 1, K - 1)
        in_zone = active & (S > L * kz - J / 2)
        zone_node = np.where(in_zone, sc.route[idx_n, kz], -1)
        cur_zone = set()
        for node in np.unique(zone_node[zone_node >= 0]):
            members = np.where(zone_node == node)[0]
            for i in members:
                if holder[node] != i and (i, kz[i]) not in viol_seen:
                    viol_seen.add((i, kz[i]))
                    violations += 1
            for a in range(len(members)):
                for b in range(a + 1, len(members)):
                    i, j = members[a], members[b]
                    same_in = kz[i] >= 1 and kz[j] >= 1 and sc.lane_id[i, kz[i] - 1] == sc.lane_id[j, kz[j] - 1]
                    same_out = sc.lane_id[i, kz[i]] == sc.lane_id[j, kz[j]]
                    if not (same_in and same_out):
                        cur_zone.add((int(i), int(j), int(node)))
        zone_events += len(cur_zone - prev_zone)
        prev_zone = cur_zone
        cur_rear = set()
        for i in np.where(active & (lead_body >= 0) & (gap_body < -0.02))[0]:
            cur_rear.add((int(min(i, lead_body[i])), int(max(i, lead_body[i]))))
        rear_events += len(cur_rear - prev_rear)
        if trace is not None and cur_rear - prev_rear:
            trace.append(dict(kind='rear', t=t, pairs=list(cur_rear - prev_rear), S=S.copy(), v=v.copy(), gap=gap.copy(),
                              lead=lead.copy(), status=status.copy(), d_entry=d_entry.copy(), v_cmd=v_cmd.copy(),
                              v_follow=v_follow.copy(), stall=sc.stall[t].copy(), active=active.copy(),
                              lf=lf.copy(), lt=lt.copy(), a_true=sc.a_true.copy(), a_nom=sc.a_nom.copy()))
        prev_rear = cur_rear
        # deadlock: nobody moved for 60 s
        prog.append((prog[-1] if prog else 0.0) + float(v.sum() * dt))
        if t > 600 and prog[-1] - prog[-601] < 0.5:
            deadlock = True
            if trace is not None:
                trace.append(dict(kind='deadlock', t=t, S=S.copy(), v=v.copy(), status=status.copy(), d_entry=d_entry.copy(),
                                  node_next=node_next.copy(), holder=holder.copy(), hold_node=hold_node.copy(),
                                  gap=gap.copy(), lead=lead.copy(), active=active.copy(), lf=lf.copy(), v_cmd=v_cmd.copy(),
                                  v_follow=v_follow.copy(), lane_count=lane_count.copy(), k_next=k_next.copy()))
            t_end = t
            break

    n_done = len(task_times)
    total = n * p.tasks_per_robot
    ticks = max(t_end, 1)
    return dict(
        policy=policy, seed=sc.seed, n=n, tasks_done=n_done, tasks_total=total,
        finished=bool(fin), deadlock=bool(deadlock), makespan=(t_end * dt),
        mean_task_time=float(np.mean(task_times)) if task_times else float("nan"),
        wait_per_task=float(wait_ticks.sum() * dt / max(n_done, 1)),
        stall_per_task=float(stall_ticks.sum() * dt / max(n_done, 1)),
        free_flow_per_task=float(sc.free_flow),
        stops_per_task=float(stops / max(n_done, 1)),
        collisions=int(zone_events + rear_events), zone_collisions=int(zone_events),
        rear_end=int(rear_events), lease_violations=int(violations),
        mean_speed=float(speed_sum / max(speed_n, 1)),
        fallback_share=float(fallback_rows / max(ctrl_rows, 1)),
        model_share=float(model_rows / max(ctrl_rows, 1)),
    )

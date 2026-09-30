"""Run the paired experiments and write the results tables.

    python -m traffic.run run --sizes 3 10 20 --seeds 30 --out traffic/results
    python -m traffic.run report --out traffic/results --md traffic/docs/RESULTS.md

`run` appends one JSON line per (label, fleet size, seed, policy) and skips lines that already
exist, so a run that is interrupted can simply be started again.
"""
import argparse
import json
import sys
import time
import warnings
from dataclasses import replace
from pathlib import Path

import numpy as np

from .control import load_controller
from .sim import Params, Scenario, run as run_sim

POLICIES = ("baseline", "predictive")


def load_rows(path):
    p = Path(path)
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []


def do_run(a):
    warnings.filterwarnings("ignore")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"runs_{a.label}.jsonl"
    done = {(r["n"], r["seed"], r["policy"]) for r in load_rows(path)}
    p = Params(buffer_s=a.buffer, gamma=a.gamma, gate_stationary=not a.no_gate,
               stall_p=a.stall, tasks_per_robot=a.tasks)
    ctrl = load_controller()
    t0 = time.time()
    for n in a.sizes:
        for seed in range(a.seed0, a.seed0 + a.seeds):
            todo = [pol for pol in a.policies if (n, seed, pol) not in done]
            if not todo:
                continue
            sc = Scenario(seed, n, p)
            for pol in todo:
                r = run_sim(sc, pol, ctrl, p)
                r["label"] = a.label
                with open(path, "a") as f:
                    f.write(json.dumps(r) + "\n")
            print(f"[{time.time() - t0:6.0f}s] label={a.label} n={n} seed={seed} done", flush=True)


# ------------------------------------------------------------------ reporting
def _pair(rows, n, metric):
    by = {}
    for r in rows:
        if r["n"] == n:
            by.setdefault(r["seed"], {})[r["policy"]] = r
    xs, ys = [], []
    for seed, d in sorted(by.items()):
        if "baseline" in d and "predictive" in d:
            b, q = d["baseline"], d["predictive"]
            if b["finished"] and q["finished"]:
                xs.append(b[metric])
                ys.append(q[metric])
    return np.array(xs), np.array(ys)


def _delay(r):
    return r["mean_task_time"] - r["free_flow_per_task"] - r["stall_per_task"]


def _ci(x, y, kind="pct", reps=4000):
    """Mean paired change of predictive vs baseline with a 95% bootstrap interval over seeds."""
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(0)
    f = (lambda a, b: 100 * (b.mean() - a.mean()) / a.mean()) if kind == "pct" else (lambda a, b: (b - a).mean())
    idx = rng.integers(0, len(x), (reps, len(x)))
    boots = np.array([f(x[i], y[i]) for i in idx])
    return f(x, y), float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))


def table(rows, label_title):
    sizes = sorted({r["n"] for r in rows})
    lines = [f"### {label_title}", "",
             "| Robots | Seeds used | Task time base -> pred (s) | Change | Traffic delay base -> pred (s) | Change | Stopped time per task base -> pred (s) | Change | Full stops per task base -> pred |",
             "|---|---|---|---|---|---|---|---|---|"]
    for n in sizes:
        rr = [dict(r, delay=_delay(r)) for r in rows if r["n"] == n]
        tb, tq = _pair(rr, n, "mean_task_time")
        db, dq = _pair(rr, n, "delay")
        wb, wq = _pair(rr, n, "wait_per_task")
        sb, sq = _pair(rr, n, "stops_per_task")
        if len(tb) == 0:
            continue
        c = _ci(tb, tq)
        d = _ci(db, dq)
        w = _ci(wb, wq)
        f = lambda t: f"{t[0]:+.1f}% ({t[1]:+.1f} to {t[2]:+.1f})"
        lines.append(f"| {n} | {len(tb)} | {tb.mean():.1f} -> {tq.mean():.1f} | {f(c)} | "
                     f"{db.mean():.1f} -> {dq.mean():.1f} | {f(d)} | {wb.mean():.1f} -> {wq.mean():.1f} | {f(w)} | "
                     f"{sb.mean():.2f} -> {sq.mean():.2f} |")
    return lines


def safety_table(rows):
    lines = ["| Robots | Policy | Runs | Deadlocked | Unfinished | Collisions | of which junction | of which rear-end | Lease violations |",
             "|---|---|---|---|---|---|---|---|---|"]
    for n in sorted({r["n"] for r in rows}):
        for pol in POLICIES:
            rr = [r for r in rows if r["n"] == n and r["policy"] == pol]
            if not rr:
                continue
            lines.append(f"| {n} | {pol} | {len(rr)} | {sum(r['deadlock'] for r in rr)} | {sum(not r['finished'] for r in rr)} | "
                         f"{sum(r['collisions'] for r in rr)} | {sum(r['zone_collisions'] for r in rr)} | "
                         f"{sum(r['rear_end'] for r in rr)} | {sum(r['lease_violations'] for r in rr)} |")
    return lines


def do_report(a):
    out = Path(a.out)
    files = sorted(out.glob("runs_*.jsonl"))
    data = {f.stem[5:]: load_rows(f) for f in files}
    print(json.dumps({k: len(v) for k, v in data.items()}))
    lines = []
    for label, rows in data.items():
        lines += [f"## Run `{label}`", ""] + table(rows, "Predictive vs stop-and-wait, paired by seed") + [""]
        lines += ["Safety, all runs:", ""] + safety_table(rows) + [""]
    Path(a.md).write_text("\n".join(lines))
    print("\n".join(lines))


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--sizes", type=int, nargs="+", default=[3, 10, 20])
    r.add_argument("--seeds", type=int, default=30)
    r.add_argument("--seed0", type=int, default=1)
    r.add_argument("--policies", nargs="+", default=list(POLICIES))
    r.add_argument("--out", default="traffic/results")
    r.add_argument("--label", default="default")
    r.add_argument("--buffer", type=float, default=None)
    r.add_argument("--gamma", type=float, default=None)
    r.add_argument("--no-gate", action="store_true")
    r.add_argument("--stall", type=float, default=0.03)
    r.add_argument("--tasks", type=int, default=5)
    r.set_defaults(fn=do_run)
    q = sub.add_parser("report")
    q.add_argument("--out", default="traffic/results")
    q.add_argument("--md", default="traffic/results/summary.md")
    q.set_defaults(fn=do_report)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()

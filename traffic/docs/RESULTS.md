# Traffic simulation results

Generated from the run files in `traffic/results/` by `python -m traffic.run report`. The tables
below are pasted from that output, not retyped. Seeds 1 to 30 were used for evaluation; the
tuning described under "Settings" used different seeds (101 to 110).

## Short version

- **Safety:** 0 collisions, 0 lease violations and 0 deadlocks in every run below (390 runs:
  180 default, 180 tuned, 30 without stalls). This is measured from true positions with
  delayed and lossy messages, sensor noise, random stalls and braking that is 90 to 110% of what
  the robot believes. It says nothing about a real lease protocol failing, because the lease here
  is an abstract arbiter.
- **Speed:** the predictive policy did **not** finish tasks faster than stop-and-wait. With default
  settings it is slower: +0.2% (3 robots), +2.0% (10) and +3.9% (20), and the 95% intervals for
  10 and 20 robots exclude zero. After tuning, the difference at 10 robots is not distinguishable
  from zero (+0.6%, -0.1 to +1.4) and at 20 robots it is still slower (+2.7%, +1.5 to +4.0).
- **What it does do:** it cuts time spent stopped by about a third at 20 robots (and by half or more
  at 3 and 10), and cuts full stops per task at 3 and 10 robots when tuned. That saved time is
  spent driving slowly instead, so total task time does not fall.
- The 20% figure in the idea PPT is **not supported** by this simulation. Do not present it as a
  result. What can be honestly said: zero collisions in simulation, and far less time stopped.

## How to read the columns

"Task time" is mean seconds per task. "Traffic delay" is task time minus the time the robot would
need alone at its plan speed minus time spent stalled, so it is the part caused by other robots.
Changes are predictive vs stop-and-wait, paired by seed, with a 95% bootstrap interval over seeds.
A positive change means predictive is slower. Runs where either policy did not finish are excluded
from the timing tables (there were none).

## Run `default`

### Predictive vs stop-and-wait, paired by seed

| Robots | Seeds used | Task time base -> pred (s) | Change | Traffic delay base -> pred (s) | Change | Stopped time per task base -> pred (s) | Change | Full stops per task base -> pred |
|---|---|---|---|---|---|---|---|---|
| 3 | 30 | 24.8 -> 24.8 | +0.2% (+0.1 to +0.3) | 0.6 -> 0.7 | +9.1% (-27.5 to +60.8) | 0.3 -> 0.1 | -66.1% (-78.5 to -56.0) | 0.20 -> 0.14 |
| 10 | 30 | 27.0 -> 27.5 | +2.0% (+1.2 to +2.8) | 3.4 -> 3.9 | +14.8% (+9.2 to +21.0) | 2.0 -> 1.0 | -51.8% (-57.5 to -45.9) | 1.25 -> 1.19 |
| 20 | 30 | 33.3 -> 34.5 | +3.9% (+2.6 to +5.3) | 8.6 -> 9.7 | +13.5% (+8.7 to +18.8) | 5.6 -> 3.7 | -32.8% (-38.0 to -27.5) | 2.99 -> 3.42 |

Safety, all runs:

| Robots | Policy | Runs | Deadlocked | Unfinished | Collisions | of which junction | of which rear-end | Lease violations |
|---|---|---|---|---|---|---|---|---|
| 3 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 10 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 10 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |

## Run `no_stalls`

### Predictive vs stop-and-wait, paired by seed

| Robots | Seeds used | Task time base -> pred (s) | Change | Traffic delay base -> pred (s) | Change | Stopped time per task base -> pred (s) | Change | Full stops per task base -> pred |
|---|---|---|---|---|---|---|---|---|
| 20 | 15 | 29.0 -> 30.2 | +4.0% (+1.9 to +6.0) | 7.7 -> 8.9 | +15.1% (+7.1 to +22.8) | 5.1 -> 3.0 | -40.0% (-48.2 to -31.3) | 2.76 -> 2.91 |

Safety, all runs:

| Robots | Policy | Runs | Deadlocked | Unfinished | Collisions | of which junction | of which rear-end | Lease violations |
|---|---|---|---|---|---|---|---|---|
| 20 | baseline | 15 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | predictive | 15 | 0 | 0 | 0 | 0 | 0 | 0 |

## Run `tuned_b0.2_g0`

### Predictive vs stop-and-wait, paired by seed

| Robots | Seeds used | Task time base -> pred (s) | Change | Traffic delay base -> pred (s) | Change | Stopped time per task base -> pred (s) | Change | Full stops per task base -> pred |
|---|---|---|---|---|---|---|---|---|
| 3 | 30 | 24.8 -> 24.8 | +0.1% (+0.0 to +0.2) | 0.6 -> 0.6 | +4.5% (-17.9 to +34.4) | 0.3 -> 0.1 | -69.0% (-82.9 to -57.9) | 0.20 -> 0.12 |
| 10 | 30 | 27.0 -> 27.2 | +0.6% (-0.1 to +1.4) | 3.4 -> 3.5 | +5.1% (-0.4 to +11.4) | 2.0 -> 0.8 | -62.4% (-66.0 to -58.7) | 1.25 -> 0.87 |
| 20 | 30 | 33.3 -> 34.1 | +2.7% (+1.5 to +4.0) | 8.6 -> 9.4 | +9.1% (+4.9 to +13.7) | 5.6 -> 3.7 | -33.3% (-39.2 to -27.2) | 2.99 -> 2.79 |

Safety, all runs:

| Robots | Policy | Runs | Deadlocked | Unfinished | Collisions | of which junction | of which rear-end | Lease violations |
|---|---|---|---|---|---|---|---|---|
| 3 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 10 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 10 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | baseline | 30 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | predictive | 30 | 0 | 0 | 0 | 0 | 0 | 0 |

## Settings

- `default`: the controller as shipped (buffer 0.5 s, risk weight 1.0), models used only when the
  robot ahead is moving.
- `tuned_b0.2_g0`: buffer 0.2 s and risk weight 0 (the conflict probability no longer scales the
  speed). Picked from a small sweep on tuning seeds 101 to 110 at 20 robots, then evaluated on the
  separate seeds 1 to 30. On the tuning seeds, using the models even behind a stopped robot was
  worse (+10% task time), so that gate stays on.
- `no_stalls`: default settings with random stalls switched off, 20 robots, 15 seeds. Predictive is
  still slower (+4.0%), so the result is not an artefact of the stall model.

## Why the speed-up did not appear (hypotheses, not tested)

1. Traffic delay is only about 8.6 s of a 33 s task at 20 robots, so a full-stop saving of about
   1 s per junction has little to act on.
2. The exclusive lease already serialises junction use, so a robot that arrives early waits either
   way. Easing off only moves the waiting from standing still to creeping.
3. The models predict clearing time for a moving peer. Peers that are accelerating from a stop look
   slow to them, so the estimate is pessimistic and the robot eases off more than it needs to.
4. The stop-and-wait baseline here brakes at the last safe moment, which is the strongest form.
   A comfortable-deceleration baseline would look worse and the gap would be smaller or negative.

## What this does not cover

One map, one fleet layout family and one set of assumed robot parameters (`traffic/sim.py`).
The lease is abstract. No lease faults, no dead robots holding junctions, no real Zenoh network,
no hardware. A different map with longer approach lanes or a lease that does not lock the whole
junction could change the speed result.


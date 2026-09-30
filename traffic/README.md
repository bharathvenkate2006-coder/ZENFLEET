# Traffic simulation: predictive speed modulation vs stop-and-wait

This folder tests the claim the rest of the repository depends on: does a robot that eases off
using the trained models finish its work faster than a robot that brakes hard and waits, and does
the safety layer keep the fleet collision-free while it does so?

Results are in [`docs/RESULTS.md`](docs/RESULTS.md). Read that file before quoting any number.

## Run it

```bash
pip install -r requirements.txt pytest
python -m pytest -q tests                                  # includes tests/test_traffic.py

# a quick look: 3 seeds, 10 robots (about 30 s)
python -m traffic.run run --sizes 10 --seeds 3 --out /tmp/zf_quick --label quick
python -m traffic.run report --out /tmp/zf_quick --md /tmp/zf_quick/summary.md

# the full experiment behind docs/RESULTS.md (about 15 min on one core)
python -m traffic.run run --sizes 3 10 20 --seeds 30 --out traffic/results --label default
```

Options for `run`: `--buffer` and `--gamma` change the controller's two caution settings,
`--no-gate` lets the models be used even when the robot ahead is stationary, `--stall` sets the
chance per 0.5 s that a robot stalls, and `--tasks` sets tasks per robot.

## What is simulated

- **Map:** a 5 x 4 grid of aisles, 4 m between nodes, two lanes per aisle (one per direction). Every
  node is a 1 m shared junction zone. 3, 10 or 20 robots, five tasks each, random destinations, no
  U-turns.
- **Robots:** payloads of 0 to 30 kg change braking (`a_max`), plan speed drifts between 0.4 and
  1.0 m/s, acceleration limit 0.8 m/s^2, true braking is 90 to 110% of what the robot believes.
  Robots stall at random exactly as in the training data (3% chance every 0.5 s, for 0.5 to 3 s).
- **Junctions:** each has an exclusive lease. A robot asks when it is 2 m away. The nearest robot at
  the head of its lane gets it. It is not granted if the exit lane already holds two robots. The
  robot learns of the grant after a message delay.
- **Communication:** every robot broadcasts position and speed at 10 Hz. Each link has a
  gamma-distributed delay (mean 80 ms, like the training data) and random 1 to 3 s outages.
- **Sensing:** position error is dead-reckoning: it resets at floor markers every 2 m and grows
  with distance. Speed, gap and peer values carry noise. Robots see the gap to a robot ahead
  directly (range sensor).
- **Measured from true positions, not from what robots believe:** two robots in one junction zone
  (unless they are following one another through it), bodies overlapping in a lane, and a robot
  entering a zone without holding its lease.

## The two policies

Both use the same safety layer, the same lease, the same following rule and the same stale-data
fallback. They differ in exactly one line, the speed suggestion (step 3 in
`src/speed_controller.py`):

| | Suggestion before the safety layer | What the robot does at a busy junction |
|---|---|---|
| Stop-and-wait | plan speed | drives at full speed, brakes as late as the safety layer allows, stops at the line, waits for the lease, accelerates again |
| Predictive | `distance / (Q90 + buffer) x (1 - conflict risk)` from the trained models | eases off so it arrives as the robot ahead clears |

The stop-and-wait baseline is deliberately the strongest version: it brakes at the last safe moment,
so it loses as little time as a robot that must stop can lose.

The models are only consulted when the robot ahead is moving (or already inside the junction).
The models were trained on peers that keep moving, and a stopped queue makes their clearing-time
estimate meaningless. `--no-gate` turns this off, and RESULTS.md reports what happens.

`traffic/control.py` is a vectorised copy of `SpeedController.step` so a whole fleet runs in one
call. `tests/test_traffic.py` checks it gives the same answer as the original.

## What is not simulated

- The lease is an abstract arbiter with message latency. The quorum lease protocol in `cnp/` is
  not part of this run, so a collision here cannot come from a lease-protocol bug.
- No lease faults: no robot dies while holding a junction, and no lease messages are lost outright.
- Turns cost no extra time, and there is no wheel, motor or Nav2 model.
- One map and one set of robot parameters. All of them are assumptions set in `sim.py`.

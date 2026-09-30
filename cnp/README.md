# ZenFleet CNP: who takes over when a robot can't finish its job

**Author: BHARATH V** | Smart India Hackathon 2026 | SIH26123 | Bharat Electronics Limited

When a robot gets stuck, runs low on battery, loses its data or just dies, somebody else should
pick up its task, and there is no central dispatcher to hand it out. This folder is how ZenFleet
does that. Robots announce the job, bid for it, and the winner must be approved by a majority of
the fleet before it starts. That last step is what stops two robots from both thinking the job is
theirs when messages get lost.

It is the Contract Net Protocol from our plan (announce, bid, award, confirm), plus the ownership
check, plus the link to the trained prediction models in the repository root.

## How it uses the prediction models

A bid is not just "how far away am I". Each robot prices the job with two things from ZenFleet:

- **Junction delay.** For every shared junction on its route, the robot asks the trained models
  (Q50 and Q90 clearing time, conflict probability) how long it would probably wait for another
  robot heading the same way. Expected delay = conflict risk x wait. A robot whose route runs into
  traffic bids higher than one with a clear road.
- **Range.** Battery percentage becomes metres it can still drive with this payload. A robot that
  cannot finish the trip with a safety margin declines instead of bidding. This part is a simple
  physics formula with constants in `config_zenfleet.json`. It is not a trained model.

And the safety layer feeds back in: if the speed controller (`src/speed_controller.py`) reports a
fallback that does not clear (stale data, lost heartbeat, blocked edge), the robot stops taking part
in the job and its peers take it over. A short glitch is ignored.

```text
speed controller output ---> safety_bridge ---> "fallback for > 2 s" ---> auction starts
trained models (Q50/Q90, risk) ---> predictors.py ---> bid cost: trip time + junction delay + range use
```

Each bid records where its prediction came from (`prediction_source`). If LightGBM or the model
files are missing, the code falls back to a plain physics estimate and says so.

## The test map

```text
    A ---4--- B ---4--- C          B and E are the shared junctions.
    |         |         |          Numbers are edge lengths in metres.
    3         3         3
    |         |         |
    D ---4--- E ---4--- F
```

Three robots (R1 at A, R2 at F heading towards D, R3 at D) and one delivery task: pick up at E,
drop at F. The layout is in `config_zenfleet.json`. `config.json` is the small original map used by
the protocol tests.

## Try it

From the repository root, install once, then work inside this folder:

```bash
pip install -r requirements.txt
cd cnp
pip install -e ".[dev]"

python -m pytest -q                                   # 57 tests
python examples/run_examples.py                       # four worked examples
python -m bidding.demo_scenarios --config config_zenfleet.json --scenario all --compare
python -m bidding.dashboard --config config_zenfleet.json   # http://127.0.0.1:8080
```

`examples/example_output.txt` holds the output of the examples, so you can see what to expect.
Run everything from inside `cnp/`, because the code looks for `config.json` and `dashboard/` in the
current folder.

## What the 17 scenarios cover

| Scenario | What happens |
|---|---|
| `basic` | An edge is blocked, the initiator asks for help, the best bidder wins |
| `dead_robot` | The robot holding the task goes silent. Peers notice and auction it |
| `no_bidders` | Nobody is eligible. Retries with a wider radius, then flagged unassigned |
| `tie` | Equal bids. The tie-break gives every robot the same winner |
| `no_confirm` | The winner disappears before confirming. The next bidder is awarded |
| `initiator_dies` | The robot that called the auction dies. A peer finishes it |
| `two_auctions` | Two tasks at once. Higher priority is taken first |
| `loss_late_duplicate` | 15% loss, 300 ms delay, 60% duplicates |
| `network_split` | The fleet splits in two. Only the majority side can own the task |
| `cancel` | The blockage clears mid-auction, so the auction is cancelled |
| `battery_drop` | The owner's battery collapses mid-task. The task is re-auctioned |
| `cargo_rescue` | A robot is stuck holding cargo. Flagged, because a physical handoff is not supported |
| `all_busy` | Every robot has a queue. Bids include the queue cost |
| `loop_limit` | Too many reassignments. Stops and asks for a human |
| `junction_congestion` | The nearest robot would run into traffic. The models change the winner |
| `low_range` | The nearest robot is too low on charge. It declines |
| `safety_fallback` | The speed controller reports a long fallback. The task moves to a peer |

## Results (short version)

Full tables are in [`docs/RESULTS.md`](docs/RESULTS.md).

- In every scenario where the robot holding the task fails, the no-reassignment baseline never
  finishes and CNP does.
- With a healthy holder, CNP costs at most about a second of handover, and finishes sooner when a
  better-placed robot exists.
- Zero duplicate executions and zero lost tasks in every run recorded here, including 40 stress runs with
  message loss. In that stress test CNP completed 39 of 40. The one miss was a task left visibly
  unassigned after retries ran out, not a silent loss.
- Handover after a safety fallback takes about 6.5 s with the default lease, and about 3 s with a
  2 s lease. The robot itself stops immediately either way.

## What this is and isn't

Be straight with yourself and the jury about these:

- **It is a protocol simulation.** Tasks progress by route length divided by speed. There is no
  physics, navigation, or motor control. The 57 tests show the coordination logic behaves as designed.
- **Real Zenoh is untested.** `transport.py` and `runtime.py` are written for eclipse-zenoh 1.x but
  have not been run against a real Zenoh network here. Try it on your three robots early.
- **The range model is a formula, not a trained model.** The junction prediction does use the trained
  models, which were trained on simulated data (see the main README).
- **CNP is greedy.** It picks the cheapest bid, not the best overall assignment.
- **Some things need real integration:** peer authentication, physical cargo handoff, and a driver
  that checks `Node.may_execute(task_id)` before moving. `docs/SAFETY.md` has the details.
- **This folder is not the 20% traffic claim.** That claim is about junction traffic, stop-and-wait
  against speed modulation, and needs the multi-robot traffic simulation, which is still to be built.

## Files

```text
bidding/predictors.py    models + energy formula -> junction delay and range for bids
bidding/safety_bridge.py speed-controller output -> "start an auction"
bidding/cost.py          eligibility and bid cost
bidding/initiator.py     the peer state machine (announce, bid, award, confirm, leases)
bidding/task_ledger.py   durable votes and ownership leases
bidding/simulator.py     seeded network and the abstract executor, plus the baseline
bidding/demo_scenarios.py  17 scenarios and the comparison table
bidding/transport.py, runtime.py   Zenoh adapter and single-robot runner
examples/run_examples.py worked examples
tests/                   protocol tests and ZenFleet integration tests
docs/                    RESULTS.md, PROTOCOL.md, SAFETY.md
```

Zenoh topics follow the ZenFleet naming: `fleet/{id}/status` (position node, speed, intended route,
battery), `map/blocked_edges`, and `cnp/...` for the auction.

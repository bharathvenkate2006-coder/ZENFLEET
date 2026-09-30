# Results

Everything below comes from running this repository on the ZenFleet warehouse map in
`config_zenfleet.json`: three robots, six nodes, two shared junctions, one delivery task, 17 scripted
situations. The simulator moves tasks forward by route length divided by 1 m/s. It models the
coordination protocol, not robot motion, so read the times as "how long the coordination takes
plus a simple drive", not as measured robot performance.

Reproduce with:

```bash
cd cnp
python -m bidding.demo_scenarios --config config_zenfleet.json --scenario all --compare
```

## What the two columns mean

- **No-reassignment baseline:** the robot that first held the task keeps it and detours around
  blocked edges on its own. If that robot dies, runs flat, or falls into safety fallback, the task is
  never finished. Nothing else about the fleet changes.
- **CNP:** the robots auction the task, the winner gets a majority-approved ownership lease, and any
  failure starts a new auction.

## All 17 scenarios

```text
scenario               no-reassignment baseline   CNP                        winner
basic                  done in  11.2 s            done in   8.7 s            R2
dead_robot             not completed              done in  12.2 s            R3
no_bidders             not completed              not completed              
tie                    done in  11.0 s            done in  11.6 s            R2
no_confirm             not completed              done in   8.6 s            R2
initiator_dies         not completed              done in   9.1 s            R2
two_auctions           done in  11.0 s            done in  11.7 s            R1, R2
loss_late_duplicate    done in  11.0 s            done in  12.1 s            R1
network_split          done in  12.5 s            done in  12.5 s            R2
cancel                 done in  11.1 s            not completed              
battery_drop           not completed              done in  16.5 s            R1
cargo_rescue           done in  11.0 s            not completed              
all_busy               done in  11.0 s            done in   8.6 s            R2
loop_limit             done in  11.0 s            not completed              
junction_congestion    not completed              done in   8.6 s            R2
low_range              done in  11.0 s            done in   8.6 s            R2
safety_fallback        not completed              done in  18.5 s            R1
```

How to read it:

- **Where the holder fails** (dead robot, battery drop, safety fallback, initiator dies, faulty
  initiator) the baseline never finishes and CNP does.
- **Where nothing is wrong** (tie, loss, network split) CNP costs between nothing and about 1.1 s of
  handover time. In `basic`, `all_busy` and `low_range` it finishes sooner only because a robot closer to
  the job exists on this map. On a map where the original holder is already the best choice, CNP is
  slightly slower, and that is the price of having a backup.
- **The four "not completed" CNP rows are intended.** `no_bidders` has nobody eligible,
  `cancel` is cancelled on purpose, `cargo_rescue` needs a physical handoff this code does not
  do, and `loop_limit` stops after too many reassignments and asks for a human.

## Bids that use the trained models

| Situation | Result |
|---|---|
| R3 is marginally cheapest on distance, but R2 is about to cross junction E | With the junction prediction on, R3's bid gets +5.3 s expected delay (conflict risk 0.64, Q90 clear time 12.7 s) and R2 wins. With it off, R3 wins. |
| R3 is next to the pickup with 11% battery and a 10% reserve | R3 declines with `insufficient_range`; R2 wins. |
| Speed controller reports a fallback that does not clear | The holder stops taking part in the job and a peer takes it over, with no duplicate execution. |

## Message loss, delay and duplication

Scenario `loss_late_duplicate`: 15% of messages dropped, up to 300 ms delay, 60% duplicated.
40 random seeds, both modes.

| | Completed | Duplicate executions | Tasks lost | Time to finish |
|---|---|---|---|---|
| No-reassignment baseline | 40 of 40 | 0 | 0 | 11.0 s every time |
| CNP | 39 of 40 | 0 | 0 | median 9.1 s, 95th percentile 12.8 s, worst 13.9 s |

The baseline is unaffected here because it never uses the network, and its holder is healthy. CNP
finished one run short: under heavy loss its retries ran out and the task was left unassigned. That is
the intended behaviour (it is flagged, not silently dropped), but it is a real failure rate to quote.

## Handover speed after a safety fallback (tuning)

The previous owner's lease has to run out before a peer may take over, which is what prevents two
robots owning one task. Shorter leases mean faster handover:

| Lease | Fallback timeout | New owner committed after fallback | Duplicates |
|---|---|---|---|
| 3.0 s (default) | 2.0 s | 6.5 s | 0 |
| 2.0 s | 1.5 s | 3.1 s | 0 |
| 1.5 s | 1.0 s | 2.6 s | 0 |

The robot itself stops moving the moment its safety layer goes to fallback. Only the paperwork of
handing over the task waits. Shorter leases need renewals to get through more often, so they are less
forgiving under message loss. I have not stress-tested the shorter settings under loss, so the defaults
stay at 3.0 s and 2.0 s.

## Tests

```text
cd cnp && python -m pytest -q     57 passed  (20 protocol tests + 37 ZenFleet integration tests)
python -m pytest -q tests         6 passed   (speed-controller safety layer, from the repository root)
```

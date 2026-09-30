# Protocol notes

Every envelope has kind, sender, timestamp, payload, and message_id. Every auction message has task_id, epoch, and key `[epoch, initiator_id]`. The initiator ID breaks equal-epoch conflicts; numeric epoch alone is insufficient. Clock units are seconds; announce also exposes bid_window_ms.

| Topic | Purpose |
|---|---|
| `cnp/announce` | Task details, trigger, origin, radius, deadline, window |
| `cnp/{task_id}/bid` | Cost, components, ETA, margin, battery, candidate route |
| `cnp/{task_id}/award` | Candidate winner, cost, confirmation deadline |
| `cnp/{task_id}/confirm` | Reservation accepted or declined; route ID |
| `cnp/{task_id}/decline` | Ineligibility reason |
| `cnp/{task_id}/cancel` | Initiator's pre-commit cancellation request |
| `cnp/{task_id}/lease_request` | Proposed owner, lease end, fencing token |
| `cnp/{task_id}/lease_grant` | Durable voter approval of exact proposal |
| `cnp/{task_id}/commit` | Majority certificate and ownership record |
| `cnp/{task_id}/release` | Completed-task tombstone |
| `fleet/{id}/status` | Liveness, node, speed, intended route, battery, priority, plus ledger and blocked-edge gossip |
| `map/blocked_edges` | Versioned blocked or cleared edge |

Deduplication uses message IDs with bounded retention. Late bids and mismatched auction keys are ignored. Deterministic ranking applies to each peer's locally accepted bid set, which can differ under loss. Lease ownership rather than bid-set agreement is the intended safety boundary.

The protocol adds quorum traffic beyond the original four core messages. Membership cannot be dynamically resized without a separate reconfiguration protocol. Radius expansion and retries are bounded; delivery and task completion are not guaranteed without eventual healthy communication and eligible capacity.

The task schema stores task_type, pickup, target, priority, deadline, payload, and carrying_cargo. All peers load the same initial task catalog. Dynamic task ingestion and external work-order systems are not included.

## Bid contents with the ZenFleet models

When the config has a `predictor` and `energy` section, a bid also carries `range_remaining_m`,
`energy_per_m_wh`, `max_conflict_risk`, `junction_detail` (per junction: peer, conflict risk, Q50 and
Q90 clearing time, expected wait) and `prediction_source` (`lightgbm` or `physics`). The cost breakdown
uses seconds for travel, queue and junction delay, and the fraction of usable range the trip would
consume for the battery term. Weights are in `config_zenfleet.json`.

A robot whose speed controller is in fallback, or whose usable range (with the safety factor) is
shorter than the trip, declines instead of bidding.

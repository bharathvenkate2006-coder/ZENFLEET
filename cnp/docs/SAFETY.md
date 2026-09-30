# Ownership safety and deployment boundary

## Assumptions

| Assumption | Required condition |
|---|---|
| Fault model | Trusted crash/recovery peers, not Byzantine senders |
| Membership | Fixed configured membership; do not change it while leases exist |
| Clock model | All clocks remain within the configured absolute skew bound |
| Durable voter state | A voter must persist a vote before publishing a grant |
| Restart | Reuse the same state directory; never erase votes to recover a live fleet |
| Identity | Authenticate peers and bind transport identity to sender IDs |
| Actuation | Independent watchdog stops motion before authorization expires |
| Task delivery | Idempotent external task effects or durable resource fencing |

Two distinct owners need intersecting majorities. An intersecting voter refuses a conflicting lease until its previous lease is certainly expired. The owner must stop before its own lease deadline minus the clock uncertainty. This is the intended ownership argument, not a formal proof or a tested production guarantee.

This Python runtime does not provide motor-level fencing. It uses wall-clock time and fails closed on detected backward jumps, but cannot detect all cross-host clock skew or suspension faults. On actual robots, use a monitored synchronized clock, bounded-drift lease design, and an independent driver watchdog. All actuation must check `Node.may_execute(task_id)` and carry the committed ownership token. A token in a Python dictionary does not physically stop another robot.

Durable writes use atomic replacement, file fsync, and directory fsync where supported. Deployment requires filesystem durability testing, single-process ownership of each robot state file, and supervised restart procedures. The runner does not enforce a process lock. Never run two processes using one robot identity.

Commit certificates contain voter IDs, not cryptographic signatures. They are acceptable only in the trusted crash-fault reference model. Before untrusted deployment, authenticate Zenoh sessions, authorize topic writers, bind sender identities, validate all schemas/ranges, and sign or independently verify grant certificates. Do not expose the demonstration fault-control HTTP server beyond localhost.

The task-completion boundary is not exactly-once transactional. A crash after physical delivery and before a durable completion tombstone can cause a later retry. Resource-side idempotency keys and recovery reconciliation are mandatory for cargo and irreversible actions. The supplied simulator cannot validate physical exactly-once behavior.

## Recovery limitations

Finite message retries cannot guarantee progress during arbitrary loss. Retry exhaustion is intentionally visible as unassigned. Once communication recovers, an operator or higher-level policy may restart failed auctions. Active-owner leases renew; failed renewals eventually stop execution. Voters do not unlock immediately on a declined offer or cancellation; conservative expiry may delay fallback.

Cancellation before ownership commit is best effort. A cancellation concurrent with a committed transfer cannot safely restore the previous owner. Recovered robots must consult the ledger, not resume their previous task automatically. Post-commit cancellation requires a separately designed stop/release protocol.

The reference includes permanent completion tombstones and heartbeat anti-entropy. High-epoch auctions, round counters, reservations, retry budgets, and log retention need a more complete durable recovery state machine before production. Reassignment counts are carried in committed state but pre-commit crash history is not fully persisted. The configured loop bound is therefore a demo guard, not a durable fleet-wide guarantee.

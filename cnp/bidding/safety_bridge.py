"""
Connects the ZenFleet speed controller (src/speed_controller.py) to the auction code.

The speed controller answers "how fast may I go right now?".  When its answer is
"not at all, my data is stale" or "my path is blocked" for long enough, the robot
should stop hoping and hand its task to someone else.  Call `apply_safety_output`
every time the speed controller produces an output:

    out = speed_controller.step(state, v_plan)
    apply_safety_output(node, out)

A short glitch (one stale message) does nothing.  A fallback that lasts longer than
`fallback_timeout` seconds in the config makes the owning robot start a new auction.
"""


def apply_safety_output(node, out, now=None, detour_available=False):
    now = node.now if now is None else now
    robot = node.robot
    reasons = out.get('reasons', [])
    in_fallback = bool(out.get('fallback_flag')) or 'blocked_edge' in reasons
    if in_fallback:
        if robot.get('safety_fallback_since') is None:
            robot['safety_fallback_since'] = now
        node.event('safety_fallback', reasons=reasons, since=robot['safety_fallback_since'])
    else:
        robot['safety_fallback_since'] = None
    # a blocked edge with no way around it needs help straight away
    robot['blocked_no_detour'] = ('blocked_edge' in reasons) and not detour_available
    return in_fallback

"""Planner/driver integrations feed these fields; peers detect missing owners."""

def local_reason(robot, task, config, now):
    if robot.get('fault'):
        return 'robot_fault'
    if not robot.get('localization_ok', True):
        return 'localization_lost'
    since = robot.get('safety_fallback_since')
    if since is not None and now - since >= config.get('fallback_timeout', 2.0):
        return 'safety_fallback'
    if robot.get('planner_failed_after_bump'):
        return 'planner_failure'
    if robot.get('blocked_no_detour'):
        return 'blocked_path'
    if robot.get('stalled_since') is not None and now - robot['stalled_since'] >= config['stall_timeout']:
        return 'stalled'
    if robot.get('remaining_range_required', 0) + config['battery_reserve'] >= robot['battery']:
        return 'battery_low'
    return None

def stale(last_seen, now, timeout):
    return now - last_seen > timeout

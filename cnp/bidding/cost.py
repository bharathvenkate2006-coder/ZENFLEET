import heapq
import math
from collections import deque

def edge_key(a, b):
    return '|'.join(sorted((a, b)))

def route(graph, start, target, blocked):
    heap = [(0.0, start, [start])]
    best = {}
    while heap:
        distance, node, path = heapq.heappop(heap)
        if node in best:
            continue
        best[node] = distance
        if node == target:
            return distance, path
        for nxt, weight in sorted(graph.get(node, {}).items()):
            if edge_key(node, nxt) not in blocked and nxt not in best:
                heapq.heappush(heap, (distance + weight, nxt, path + [nxt]))
    return math.inf, []

def hops(graph, start, target):
    queue, seen = deque([(start, 0)]), {start}
    while queue:
        node, depth = queue.popleft()
        if node == target:
            return depth
        for nxt in graph.get(node, {}):
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, depth + 1))
    return math.inf

def evaluate(robot, task, graph, blocked, config, now, predictor=None, peers=None, robot_id=None):
    """Return (bid, None) if the robot may bid, else (None, reason).

    Without a predictor this is the original range-unit cost. With a predictor that has an
    energy model, battery becomes metres of range and the ZenFleet junction prediction adds
    an expected-delay term.
    """
    if task.get('carrying_cargo'):
        return None, 'rescue_required'
    if robot.get('critical') or not robot.get('localization_ok', True) or not robot.get('comms_ok', True):
        return None, 'critical_or_stale'
    if robot.get('safety_fallback_since') is not None:
        return None, 'safety_fallback'
    if now > robot.get('telemetry_valid_until', math.inf):
        return None, 'stale_telemetry'
    if robot.get('fault') or robot['capacity'] < task['payload']:
        return None, 'fault_or_capacity'
    reach, first = route(graph, robot['node'], task['pickup'], blocked)
    finish, second = route(graph, task['pickup'], task['target'], blocked)
    path = first + second[1:]
    if not math.isfinite(reach + finish):
        return None, 'path_battery_or_deadline'

    if predictor is not None and predictor.has_energy:
        trip = reach + finish
        speed = predictor.speed(robot)
        e_per_m = predictor.energy_per_m(robot, task['payload'])
        range_m = predictor.range_m(robot, task['payload'])
        factor = predictor.energy.get('safety_factor', 1.25)
        if trip * factor > range_m:
            return None, 'insufficient_range'
        jd = {'expected_delay_s': 0.0, 'max_risk': 0.0, 'junctions': [], 'source': predictor.source}
        if predictor.use_congestion and peers:
            jd = predictor.junction_delay(robot_id, robot, path, peers, now, task['payload'])
        travel_s = trip / speed
        queue = robot.get('queue_load', 0)
        eta = travel_s + queue + jd['expected_delay_s']
        if now + eta > task['deadline']:
            return None, 'path_battery_or_deadline'
        parts = {'travel': travel_s, 'queue': queue, 'battery': trip / range_m,
                 'congestion': jd['expected_delay_s']}
        cost = round(sum(config['weights'][k] * v for k, v in parts.items()) * 1000000)
        return {'total_cost': cost, 'cost_breakdown': parts, 'eta': eta,
                'battery_margin': range_m - trip, 'battery': robot['battery'], 'route': path,
                'range_remaining_m': round(range_m, 1), 'energy_per_m_wh': round(e_per_m, 4),
                'max_conflict_risk': jd['max_risk'], 'junction_detail': jd['junctions'],
                'prediction_source': jd['source']}, None

    margin = robot['battery'] - reach - finish - config['battery_reserve']
    eta = reach + finish + robot.get('queue_load', 0)
    if not math.isfinite(eta) or margin <= 0 or now + eta > task['deadline']:
        return None, 'path_battery_or_deadline'
    parts = {'travel': reach, 'queue': robot.get('queue_load', 0),
             'battery': 1.0 / margin, 'congestion': robot.get('congestion', 0)}
    cost = round(sum(config['weights'][k] * value for k, value in parts.items()) * 1000000)
    return {'total_cost': cost, 'cost_breakdown': parts, 'eta': eta,
            'battery_margin': margin, 'battery': robot['battery'],
            'route': path}, None

def bid_key(bid):
    # Robot IDs use lexicographic ordering (R10 precedes R2).
    return bid['total_cost'], -bid['battery'], bid['bidder_id']

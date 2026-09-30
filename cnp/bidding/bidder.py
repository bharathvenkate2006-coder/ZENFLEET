from .cost import evaluate, hops

def make_bid(node, auction):
    task = auction['task']
    if hops(node.config['graph'], node.robot['node'], auction['origin_node']) > auction['radius']:
        return None, 'outside_radius'
    if node.reservation and node.reservation[1] > node.now:
        return None, 'reserved'
    for tid in node.ledger.entries:
        if node.ledger.can_execute(tid, node.id, node.now):
            return None, 'already_executing'
    bid, reason = evaluate(node.robot, task, node.config['graph'], node.blocked,
                           node.config, node.now, node.predictor, node.peer_info, node.id)
    if bid:
        bid['bidder_id'] = node.id
    return bid, reason

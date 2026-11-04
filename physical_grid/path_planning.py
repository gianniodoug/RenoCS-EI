"""Existing route planner reused for the physical 25-node, one-metre-edge grid.
physical_route() selects a route or a single manoeuvre trial; it never moves
motors or updates measured location. There is no extra external-base node.
"""
import heapq
import math
from typing import Any, Dict, List, Optional, Tuple
from grid_map import GridMap, edge_key

ANGLES = {'E': 0, 'N': 90, 'W': 180, 'S': 270}
EPS = 1e-9


def physical_route(start, destination, heading, mode='AUTO'):
    """Setup only: 25-node route or one manoeuvre trial; never updates location."""
    grid = GridMap()
    if mode not in ('STOP', 'LEFT', 'RIGHT', 'STRAIGHT', 'AUTO'):
        raise ValueError('MODE must be STOP, LEFT, RIGHT, STRAIGHT or AUTO')
    if start not in grid.adjacency or destination not in grid.adjacency or heading not in ANGLES:
        raise ValueError('Use nodes A..Y and heading N/E/S/W')
    def neighbour(node, direction):
        result = next((n for n in grid.adjacency[node] if grid.direction(node, n) == direction), None)
        if result is None:
            raise ValueError('No grid edge from '+node+' towards '+direction)
        return result
    if mode != 'AUTO':
        path = [start, neighbour(start, heading)]
        if mode != 'STOP':
            direction = next(d for d in ANGLES if turn_command(heading, d) == mode)
            path.append(neighbour(path[1], direction))
        return grid, path
    blocked = [(start, n) for n in grid.adjacency[start]
               if turn_command(heading, grid.direction(start, n)) == 'UTURN']
    plan = plan_mission_from_state(grid, dict(current_node=start, at_node=True,
             orientation=heading), dict(type='node', node=destination), blocked)
    if not plan['feasible']:
        raise ValueError(plan['reason'])
    return grid, plan['path'] or [start]


def turn_command(current: Optional[str], following: Optional[str]) -> str:
    if current not in ANGLES or following not in ANGLES:
        return 'ALIGN_REQUIRED'  # curved base link / unknown physical heading
    d = (ANGLES[following] - ANGLES[current]) % 360
    return {0: 'STRAIGHT', 90: 'LEFT', 180: 'UTURN', 270: 'RIGHT'}[d]


def _position(grid: GridMap, state: Dict[str, Any]):
    """(node, or (u,v,p)) in the directed segment reference."""
    if state.get('at_node', state.get('from_node') is None):
        node = state.get('current_node')
        if node not in grid.adjacency:
            raise ValueError('Unknown current node')
        return node
    u, v = state.get('from_node'), state.get('to_node')
    p = float(state['progress'])
    if v not in grid.adjacency.get(u, set()) or not math.isfinite(p) or not 0 <= p <= 1:
        raise ValueError('Invalid current segment/progress')
    if p < EPS:
        return u
    if p > 1 - EPS:
        return v
    return (u, v, p)


def _target(grid: GridMap, target: Dict[str, Any]):
    typ = target.get('type', 'node')
    if typ == 'node':
        node = target['node']
        if node not in grid.adjacency:
            raise ValueError('Unknown target node')
        return node
    if typ == 'edge':
        u, v = target['edge']
        p = float(target['progress'])
        if v not in grid.adjacency.get(u, set()) or not math.isfinite(p) or not 0 <= p <= 1:
            raise ValueError('Invalid target edge/progress')
        if p < EPS:
            return u
        if p > 1 - EPS:
            return v
        return (u, v, p)
    raise ValueError('Unknown target type')


def _weight(grid: GridMap, u: str, v: str) -> float:
    length = grid.length_m(u, v)
    return float(length) if length is not None else 1.0


def _build_graph(grid: GridMap, start, goal, blocked):
    """Split every affected edge ONCE, including start+goal on the same edge."""
    graph = {n: {} for n in grid.adjacency}
    marks = {}
    for tag, pos in [('__POSITION__', start), ('__GOAL__', goal)]:
        if isinstance(pos, tuple):
            u, v, p = pos
            key = edge_key(u, v)
            canonical_p = p if (u, v) == key else 1 - p
            marks.setdefault(key, []).append((canonical_p, tag))
    segments = {}  # (a,b) -> (canonical endpoints, fractions)
    for u in sorted(grid.adjacency):
        for v in sorted(grid.adjacency[u]):
            if u >= v:
                continue
            if edge_key(u, v) in blocked:
                continue
            seq = [(0., u)] + sorted(marks.get((u, v), [])) + [(1., v)]
            length = _weight(grid, u, v)
            for (pa, a), (pb, b) in zip(seq, seq[1:]):
                cost = (pb - pa) * length
                graph.setdefault(a, {})[b] = cost
                graph.setdefault(b, {})[a] = cost
                segments[(a, b)] = (u, v, pa, pb)
                segments[(b, a)] = (v, u, 1-pb, 1-pa)
    # If two virtual nodes are at exactly same point, allow zero-cost crossing.
    return graph, segments


def _shortest(graph, start, goal):
    if start == goal:
        return [start], 0.
    dist = {start: 0.}
    prev = {}
    heap = [(0., start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u] + EPS:
            continue
        if u == goal:
            break
        for v, w in graph.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, math.inf) - EPS:
                dist[v], prev[v] = nd, u
                heapq.heappush(heap, (nd, v))
    if goal not in dist:
        return None, math.inf
    path, at = [goal], goal
    while at != start:
        at = prev[at]
        path.append(at)
    return path[::-1], dist[goal]


def plan_mission_from_state(grid: GridMap, robot_state: Dict[str, Any],
                            target: Dict[str, Any], blocked_edges=None) -> Dict[str, Any]:
    """Plan from localizer.snapshot() to node or partial edge target.

    blocked_edges: iterable of (u,v) segments, bidirectional. Doesn't mutate map.
    The returned expected_end_state is a PREDICTION, not a localization update.
    """
    start, goal = _position(grid, robot_state), _target(grid, target)
    blocked = {edge_key(*e) for e in (blocked_edges or [])}
    for e in blocked:
        if e[1] not in grid.adjacency.get(e[0], set()):
            raise ValueError('Blocked segment not in map')
    fail = lambda reason: {'feasible': False, 'reason': reason, 'path': None,
                            'commands': [], 'detailed_steps': [], 'total_cost': math.inf,
                            'cost_unit': 'm' if all(grid.length_m(a,b) is not None for a in grid.adjacency for b in grid.adjacency[a]) else 'nominal_or_mixed',
                            'expected_end_state': None}
    if isinstance(start, tuple) and edge_key(*start[:2]) in blocked:
        return fail('Robot is inside blocked edge: STOP, recover first (no teleport).')
    if isinstance(goal, tuple) and edge_key(*goal[:2]) in blocked:
        return fail('Destination is inside blocked edge.')
    # Physical coincidence (including opposite references of the same edge):
    # no movement and no artificial heading change.
    if (start == goal or (isinstance(start, tuple) and isinstance(goal, tuple)
        and edge_key(*start[:2]) == edge_key(*goal[:2])
        and abs((start[2] if start[:2] == edge_key(*start[:2]) else 1-start[2])
              - (goal[2] if goal[:2] == edge_key(*goal[:2]) else 1-goal[2])) < EPS)):
        return {'feasible': True, 'reason': 'Already at destination',
                'path': [], 'commands': [], 'detailed_steps': [],
                'total_cost': 0., 'cost_unit': 'm' if all(grid.length_m(a,b) is not None for a in grid.adjacency for b in grid.adjacency[a]) else 'nominal_or_mixed',
                'expected_end_state': dict(robot_state)}
    graph, segments = _build_graph(grid, start, goal, blocked)
    sid = '__POSITION__' if isinstance(start, tuple) else start
    gid = '__GOAL__' if isinstance(goal, tuple) else goal
    path, cost = _shortest(graph, sid, gid)
    if path is None:
        return fail('No path avoiding blocked segments.')
    steps = []
    heading = robot_state.get('orientation')
    for a, b in zip(path, path[1:]):
        u, v, p0, p1 = segments[(a, b)]
        next_heading = grid.direction(u, v)
        action = turn_command(heading, next_heading)
        step = {'at_node': a, 'next_node': b, 'segment': (u, v),
                'start_progress': round(p0, 8), 'end_progress': round(p1, 8),
                'action': action, 'heading': next_heading,
                'cost': graph[a][b], 'requires_physical_confirmation': True}
        steps.append(step)
        heading = next_heading
    # A partial target stays partial; a target node is marked at_node=True.
    if isinstance(goal, tuple):
        u, v, p = goal
        last_seg = steps[-1]['segment'] if steps else (u, v)
        e0, e1 = last_seg
        final_progress = p if (e0, e1) == (u, v) else 1-p
        end = {'current_node': e0, 'from_node': e0,
               'to_node': e1, 'progress': final_progress, 'orientation': heading,
               'at_node': False}
    else:
        end = {'current_node': goal, 'from_node': None, 'to_node': None,
               'progress': 0., 'orientation': heading, 'at_node': True}
    metric = all(grid.length_m(a,b) is not None for a in grid.adjacency for b in grid.adjacency[a])
    return {'feasible': True, 'reason': 'OK', 'path': path,
            'commands': [s['action'] for s in steps], 'detailed_steps': steps,
            'total_cost': round(cost, 6), 'cost_unit': 'm' if metric else 'nominal_or_mixed',
            'expected_end_state': end}

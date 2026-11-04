"""Physical grid: 4 x 4 one-metre cells, exactly 25 nodes and 40 unit edges.
Node naming convention (north at top): A B C D E / F G H I J / K L M N O /
P Q R S T / U V W X Y. Coordinates are metres, x east and y north.
There is no extra external-base node.
"""
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

ROWS, COLS = 5, 5
NODES = tuple(chr(ord('A') + i) for i in range(25))
Edge = Tuple[str, str]


def edge_key(a: str, b: str) -> Edge:
    if a == b:
        raise ValueError("An edge needs two different nodes")
    return tuple(sorted((a, b)))


@dataclass
class GridMap:
    coordinates: Dict[str, Tuple[int, int]] = field(default_factory=dict)
    adjacency: Dict[str, set] = field(default_factory=dict)
    edge_lengths_m: Dict[Edge, float] = field(default_factory=dict)

    def __post_init__(self):
        if not self.coordinates:
            for i, name in enumerate(NODES):
                self.coordinates[name] = (i % COLS, ROWS - 1 - i // COLS)
        if not self.adjacency:
            self.adjacency = {node: set() for node in self.coordinates}
            for row in range(ROWS):
                for col in range(COLS):
                    name = NODES[row * COLS + col]
                    if col + 1 < COLS:
                        self.add_edge(name, NODES[row * COLS + col + 1], 1.0)
                    if row + 1 < ROWS:
                        self.add_edge(name, NODES[(row + 1) * COLS + col], 1.0)

    def add_edge(self, a: str, b: str, length_m: Optional[float] = None):
        if a not in self.coordinates or b not in self.coordinates or a == b:
            raise ValueError('Invalid endpoints')
        self.adjacency[a].add(b)
        self.adjacency[b].add(a)
        if length_m is not None:
            self.set_length(a, b, length_m)

    def set_length(self, a: str, b: str, length_m: float):
        if b not in self.adjacency.get(a, set()):
            raise ValueError('Not a connected segment')
        if not isinstance(length_m, (int, float)) or not 0 < length_m < float('inf'):
            raise ValueError('Length must be finite and positive')
        self.edge_lengths_m[edge_key(a, b)] = float(length_m)

    def length_m(self, a: str, b: str) -> Optional[float]:
        return self.edge_lengths_m.get(edge_key(a, b))

    def direction(self, a: str, b: str) -> Optional[str]:
        """Cardinal heading of a one-metre grid edge."""
        if b not in self.adjacency.get(a, set()):
            raise ValueError('Not connected')
        x1, y1 = self.coordinates[a]
        x2, y2 = self.coordinates[b]
        return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[(x2-x1, y2-y1)]

    def as_dict(self):
        return {
            'nodes': {n: list(p) for n, p in self.coordinates.items()},
            'edges': [{'from': a, 'to': b, 'length_m': self.length_m(a, b)}
                      for a in sorted(self.adjacency)
                      for b in sorted(self.adjacency[a]) if a < b],
            'units': 'm',
        }


if __name__ == '__main__':
    grid = GridMap()
    print(f'{len(grid.coordinates)} nodes, {sum(map(len, grid.adjacency.values())) // 2} edges')
    print('All grid edges are 1.0 m; there is no external START node.')

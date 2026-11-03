"""Topological model of the VAC grid, based on the supplied photograph.

Provisional assumption: 5x5 grid nodes A..Y, plus an external base START
(displayed as S). Grid coordinates are NOT metres. The photo alone does not
establish the length of any segment, including the curved START--Y segment.
"""
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

ROWS, COLS = 5, 5
BASE = "START"  # Display name S; 'S' already belongs to the 25 grid nodes.
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
            # The external base position is only symbolic. Curved link START--Y.
            self.coordinates[BASE] = (5, -1)
        if not self.adjacency:
            self.adjacency = {node: set() for node in self.coordinates}
            for row in range(ROWS):
                for col in range(COLS):
                    name = NODES[row * COLS + col]
                    if col + 1 < COLS:
                        self.add_edge(name, NODES[row * COLS + col + 1])
                    if row + 1 < ROWS:
                        self.add_edge(name, NODES[(row + 1) * COLS + col])
            self.add_edge(BASE, 'Y')

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
        """Cardinal heading of a STRAIGHT grid edge, None on curved START--Y."""
        if b not in self.adjacency.get(a, set()):
            raise ValueError('Not connected')
        if BASE in (a, b):
            return None
        x1, y1 = self.coordinates[a]
        x2, y2 = self.coordinates[b]
        return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[(x2-x1, y2-y1)]

    def as_dict(self):
        return {
            'nodes': {n: list(p) for n, p in self.coordinates.items()},
            'edges': [{'from': a, 'to': b, 'length_m': self.length_m(a, b)}
                      for a in sorted(self.adjacency)
                      for b in sorted(self.adjacency[a]) if a < b],
            'base_id': BASE, 'base_display': 'S',
            'units': 'grid coordinates; distances in metres only when measured',
        }


if __name__ == '__main__':
    grid = GridMap()
    print(f'{len(grid.coordinates)} nodes, {sum(map(len, grid.adjacency.values())) // 2} edges')
    print('Start neighbours:', grid.adjacency[BASE])

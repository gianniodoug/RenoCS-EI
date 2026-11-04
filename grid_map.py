"""Modello topologico della griglia 5x5 con sistema di riferimento Cartesiano.

I nodi sono stringhe di due cifre 'XY':
- '00' è l'origine in basso a sinistra (x=0, y=0).
- '40' è lo spigolo in basso a destra (x=4, y=0).
- '04' è lo spigolo in alto a sinistra (x=0, y=4).
- '44' è lo spigolo in alto a destra (x=4, y=4).
"""
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

ROWS, COLS = 5, 5
BASE = "START"  # Base esterna opzionale collegata a '40' (ex nodo 'Y')
NODES = tuple(f"{x}{y}" for y in range(ROWS) for x in range(COLS))
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
            for x in range(COLS):
                for y in range(ROWS):
                    self.coordinates[f"{x}{y}"] = (x, y)
            # Posizione simbolica della base esterna (collegata allo spigolo in basso a destra '40')
            self.coordinates[BASE] = (5, -1)

        if not self.adjacency:
            self.adjacency = {node: set() for node in self.coordinates}
            for x in range(COLS):
                for y in range(ROWS):
                    name = f"{x}{y}"
                    if x + 1 < COLS:
                        self.add_edge(name, f"{x + 1}{y}")
                    if y + 1 < ROWS:
                        self.add_edge(name, f"{x}{y + 1}")
            # Mantiene la compatibilità con la base esterna collegandola a '40' (in basso a destra)
            self.add_edge(BASE, '40')

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
        """Direzione cardinale di un segmento dritto ('E', 'W', 'N', 'S')."""
        if b not in self.adjacency.get(a, set()):
            raise ValueError('Not connected')
        if BASE in (a, b):
            return None
        x1, y1 = self.coordinates[a]
        x2, y2 = self.coordinates[b]
        return {(1, 0): 'E', (-1, 0): 'W', (0, 1): 'N', (0, -1): 'S'}[(x2 - x1, y2 - y1)]

    def as_dict(self):
        return {
            'nodes': {n: list(p) for n, p in self.coordinates.items()},
            'edges': [{'from': a, 'to': b, 'length_m': self.length_m(a, b)}
                      for a in sorted(self.adjacency)
                      for b in sorted(self.adjacency[a]) if a < b],
            'base_id': BASE, 'base_display': 'S',
            'units': 'cartesian grid coordinates (x, y); distances in metres only when measured',
        }


if __name__ == '__main__':
    grid = GridMap()
    print(f'{len(grid.coordinates)} nodes, {sum(map(len, grid.adjacency.values())) // 2} edges')
    print('Origine in basso a sinistra (00) vicini:', sorted(grid.adjacency['00']))
    print('Spigolo in alto a destra (44) vicini:', sorted(grid.adjacency['44']))

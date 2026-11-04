"""VAC event-based localization on an already known grid.

This module does not infer motion from images. It consumes validated events:
- entering a known segment (begin_segment);
- progress estimated from odometry (update_progress);
- confirmed arrival at its endpoint (confirm_arrival);
- reversing direction on a segment (reverse_direction).
Camera intersection events must be debounced and associated with the map.
"""
from dataclasses import dataclass
from typing import Optional
from grid_map import GridMap, BASE


@dataclass
class RobotLocalizer:
    grid: GridMap
    current_node: str = BASE
    from_node: Optional[str] = None
    to_node: Optional[str] = None
    progress: float = 0.0
    orientation: Optional[str] = None

    def begin_segment(self, next_node: str):
        if self.from_node is not None:
            raise RuntimeError('Already traversing a segment')
        if next_node not in self.grid.adjacency.get(self.current_node, set()):
            raise ValueError('Destination is not adjacent')
        self.from_node, self.to_node = self.current_node, next_node
        self.progress = 0.0
        self.orientation = self.grid.direction(self.from_node, self.to_node)

    def update_progress(self, fraction: float):
        """Absolute position fraction in the current travel direction, from odometry."""
        if self.from_node is None:
            raise RuntimeError('Not travelling on a segment')
        if not isinstance(fraction, (int, float)) or not 0 <= fraction <= 1:
            raise ValueError('Progress must be a fraction between 0 and 1')
        self.progress = float(fraction)

    def update_distance_m(self, distance_m: float):
        """Set progress from distance since entry; requires measured edge length."""
        if self.from_node is None:
            raise RuntimeError('Not travelling on a segment')
        length = self.grid.length_m(self.from_node, self.to_node)
        if length is None:
            raise RuntimeError('Real length not yet measured for this segment')
        if not isinstance(distance_m, (int, float)) or not 0 <= distance_m <= length:
            raise ValueError('Distance outside segment')
        self.progress = float(distance_m) / length

    def reverse_direction(self):
        """Reverse travel reference AFTER a confirmed physical U-turn."""
        if self.from_node is None:
            raise RuntimeError('Not travelling on a segment')
        self.from_node, self.to_node = self.to_node, self.from_node
        self.progress = 1.0 - self.progress
        self.orientation = self.grid.direction(self.from_node, self.to_node)

    def confirm_arrival(self, node: str):
        """Call only when physical arrival at the expected node is VERIFIED."""
        if self.to_node != node or self.from_node is None:
            raise ValueError('Unexpected arrival: localization needs reconciliation')
        self.current_node = node
        self.from_node = self.to_node = None
        self.progress = 0.0
        # Orientation remains previous heading (None on the curved base link).

    def obstacle_position(self, distance_ahead_m: float):
        """Return position of obstacle on current edge, or None if not localizable.

        Assumes obstacle lies on the current edge; no lateral/diagonal estimate.
        """
        if self.from_node is None:
            return None
        length = self.grid.length_m(self.from_node, self.to_node)
        if length is None or not isinstance(distance_ahead_m, (int, float)) or distance_ahead_m < 0:
            return None
        fraction = self.progress + distance_ahead_m / length
        if fraction > 1:
            return None  # obstacle may be beyond next node; do not assign false location
        return {'edge': [self.from_node, self.to_node], 'fraction': round(fraction, 4),
                'distance_ahead_m': float(distance_ahead_m), 'estimated': True}

    def snapshot(self):
        return {'current_node': self.current_node, 'from_node': self.from_node,
                'to_node': self.to_node, 'progress': self.progress,
                'orientation': self.orientation,
                'at_node': self.from_node is None}

"""Explicit bridge from measured encoder counts to the T.7/T.3 state.

Call only AFTER navigation.maneuver_result(True, True) was validated with
physical line reacquisition; do not call while turning or on the base curve
until its length is physically measured and odometry is validated.
"""
from encoder_odometry import SegmentProgressEstimator


class NavigationOdometryAdapter:
    def __init__(self, nav, calibration):
        self.nav = nav
        self.calibration = calibration
        self.segment_estimator = None

    def begin_following(self, encoder_counts):
        if self.nav.phase != 'FOLLOWING':
            raise RuntimeError('Nav must be following a line')
        loc = self.nav.localizer
        length = self.nav.grid.length_m(loc.from_node, loc.to_node)
        if length is None:
            raise RuntimeError('Measure physical segment length first')
        self.segment_estimator = SegmentProgressEstimator(self.calibration, length)
        self.segment_estimator.reset(encoder_counts)

    def update(self, encoder_counts):
        if self.nav.phase != 'FOLLOWING' or self.segment_estimator is None:
            raise RuntimeError('Not tracking a segment')
        estimate = self.segment_estimator.update(encoder_counts)
        # Pre-check partial delivery overshoot: navigation has its own guard.
        result = self.nav.progress_update(estimate['progress'])
        return {'nav_event': result, 'estimated_progress': estimate['progress'],
                'estimated_distance_m': estimate['distance_m'],
                'disagreement_m': estimate['sample'].disagreement_m}

    def clear(self):
        self.segment_estimator = None

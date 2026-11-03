"""Odometry from four *cumulative* Arduino quadrature encoder counters.

All geometric/calibration parameters are explicit. No wheel motion is commanded.
The firmware supplies counters in FR, FL, RR, RL order; counter directions
must be verified on the real robot before enabling physical navigation.
"""
from dataclasses import dataclass
from math import isfinite, pi
from statistics import median

WHEELS = ('FR', 'FL', 'RR', 'RL')


@dataclass(frozen=True)
class EncoderCalibration:
    # Number of count increments per complete wheel revolution (NOT necessarily
    # the encoder's advertised PPR; gear ratio and interrupt mode matter).
    ticks_per_wheel_rev: float
    wheel_diameter_m: float
    # Raw encoder increment sign when the corresponding wheel rolls FORWARD.
    forward_signs: tuple  # (FR, FL, RR, RL), each +1 or -1
    # Relative yaw from differential-drive approximation (wheel separation).
    # Calibrate on robot: skid-steer slip can dominate this geometric estimate.
    effective_track_width_m: float

    def __post_init__(self):
        if (not all(isfinite(v) and v > 0 for v in (self.ticks_per_wheel_rev,
                self.wheel_diameter_m, self.effective_track_width_m))):
            raise ValueError('Positive finite geometry required')
        if len(self.forward_signs) != 4 or any(type(x) is not int or x not in (-1, 1)
                                                for x in self.forward_signs):
            raise ValueError('forward_signs must be 4 verified values of +/-1')

    @property
    def metres_per_tick(self):
        return pi * self.wheel_diameter_m / self.ticks_per_wheel_rev


def _counts(counts):
    if len(counts) != 4 or any(type(x) is not int for x in counts):
        raise ValueError('Expected 4 integer cumulative encoder counts (FR, FL, RR, RL)')
    return tuple(counts)


def _difference(now, before):
    # Arduino long is signed 32-bit; accommodate overflow between readings.
    return tuple(((n - b + (1 << 31)) % (1 << 32)) - (1 << 31)
                 for n, b in zip(_counts(now), _counts(before)))


@dataclass(frozen=True)
class OdometrySample:
    distance_m: float  # signed forward distance estimate, mean of left/right sides
    yaw_rad: float  # positive CCW, negative CW; APPROXIMATION
    right_m: float
    left_m: float
    wheel_m: tuple
    disagreement_m: float  # disagreement between front/rear on each side


class EncoderOdometry:
    def __init__(self, calibration):
        self.cal = calibration
        self._last = None
        self.total_distance_m = 0.0
        self.total_yaw_rad = 0.0

    def reset(self, cumulative_counts):
        self._last = _counts(cumulative_counts)
        self.total_distance_m = 0.0
        self.total_yaw_rad = 0.0

    def update(self, cumulative_counts):
        now = _counts(cumulative_counts)
        if self._last is None:
            self.reset(now)
            return OdometrySample(0., 0., 0., 0., (0.,)*4, 0.)
        raw = _difference(now, self._last)
        self._last = now
        wheel_m = tuple(d * sign * self.cal.metres_per_tick
                        for d, sign in zip(raw, self.cal.forward_signs))
        right = (wheel_m[0] + wheel_m[2]) / 2
        left = (wheel_m[1] + wheel_m[3]) / 2
        distance = (right + left) / 2
        yaw = (right - left) / self.cal.effective_track_width_m
        disagreement = max(abs(wheel_m[0] - wheel_m[2]),
                           abs(wheel_m[1] - wheel_m[3]))
        self.total_distance_m += distance
        self.total_yaw_rad += yaw
        return OdometrySample(distance, yaw, right, left, wheel_m, disagreement)


class SegmentProgressEstimator:
    """Compute fraction of a known segment, starting at its departure node.

    reset() must be called after the confirmed node departure; turns and reverse
    manoeuvres must NOT accumulate into the segment distance.
    """
    def __init__(self, cal, segment_length_m):
        if not isfinite(segment_length_m) or segment_length_m <= 0:
            raise ValueError('Segment length must be measured in metres')
        self.length_m = segment_length_m
        self.odometry = EncoderOdometry(cal)

    def reset(self, counts):
        self.odometry.reset(counts)

    def update(self, counts):
        sample = self.odometry.update(counts)
        distance = self.odometry.total_distance_m
        if distance < -0.02 or distance > self.length_m + 0.02:
            raise RuntimeError('Segment odometry out of bounds; stop and reconcile position')
        return {'progress': min(1., max(0., distance / self.length_m)),
                'distance_m': distance, 'sample': sample}


class EncoderTurnMonitor:
    """Offline turn-angle monitor; DOES NOT command motors or verify line reacquisition.

    Call begin(counts, action) with robot at rest; then update(new_counts).
    Positive left turns require positive differential yaw. With physical skid-steer
    the effective track width is empirical; calibration must be measured.
    """
    def __init__(self, calibration, tolerance_deg=5., max_disagreement_m=0.08):
        if not (0 <= tolerance_deg < 45):
            raise ValueError('Invalid angular tolerance')
        self.odo = EncoderOdometry(calibration)
        self.tolerance_deg = tolerance_deg
        self.max_disagreement_m = max_disagreement_m
        self.action = None
        self.goal_deg = None
        self.status = 'IDLE'

    def begin(self, cumulative_counts, action):
        if action not in ('LEFT', 'RIGHT', 'UTURN'):
            raise ValueError('Turn monitor supports LEFT, RIGHT and UTURN')
        self.odo.reset(cumulative_counts)
        self.action = action
        self.goal_deg = {'LEFT': 90., 'RIGHT': -90., 'UTURN': 180.}[action]
        self.status = 'TRACKING'

    def update(self, cumulative_counts):
        if self.status not in ('TRACKING', 'ANGLE_REACHED'):
            raise RuntimeError('Call begin before monitoring an encoder turn')
        s = self.odo.update(cumulative_counts)
        deg = self.odo.total_yaw_rad * 180 / pi
        target_sign = 1 if self.goal_deg > 0 else -1
        if s.disagreement_m > self.max_disagreement_m:
            self.status = 'FAULT'
            return {'status': self.status, 'angle_deg': deg, 'reason': 'Encoder disagreement'}
        # Detect significant movement opposite to requested rotation.
        if target_sign * deg < -self.tolerance_deg:
            self.status = 'FAULT'
            return {'status': self.status, 'angle_deg': deg, 'reason': 'Wrong rotation direction'}
        if target_sign * deg >= abs(self.goal_deg) - self.tolerance_deg:
            self.status = 'ANGLE_REACHED'
        return {'status': self.status, 'angle_deg': deg, 'target_deg': self.goal_deg,
                'remaining_deg': max(0., abs(self.goal_deg) - target_sign * deg),
                'line_reacquired': False, 'physical_turn_confirmed': False}

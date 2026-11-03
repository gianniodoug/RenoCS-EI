"""Encoder feedback turn controller. No motor movement on import.

Angles are estimates; use only after verifying encoder signs, wheel ticks and
calibrating effective track width. Controller never signals line reacquired.
"""
from dataclasses import dataclass
from math import degrees, isfinite
import time
from encoder_odometry import EncoderOdometry


class TurnFault(RuntimeError):
    pass


@dataclass(frozen=True)
class TurnSettings:
    fast_pwm: int = 65
    slow_pwm: int = 42
    slow_at_remaining_deg: float = 22.0
    stop_early_deg: float = 3.0
    max_turn_s: float = 7.0
    no_motion_s: float = 0.7
    min_motion_deg: float = 0.25
    poll_s: float = 0.05
    max_encoder_disagreement_m: float = 0.06
    max_sample_jump_deg: float = 25.0

    def __post_init__(self):
        if not (0 < self.slow_pwm <= self.fast_pwm <= 100):
            raise ValueError('PWM out of safe configuration range (not a verified physical safe speed)')
        for v in (self.slow_at_remaining_deg, self.max_turn_s, self.no_motion_s,
                  self.min_motion_deg, self.poll_s, self.max_encoder_disagreement_m,
                  self.max_sample_jump_deg):
            if not isfinite(v) or v <= 0:
                raise ValueError('Invalid turn setting')
        if not (0 <= self.stop_early_deg < self.slow_at_remaining_deg < 90):
            raise ValueError('Invalid threshold order')
        if not self.poll_s < self.no_motion_s < self.max_turn_s:
            raise ValueError('Invalid timing')


class ClosedLoopTurn:
    """Must run from one exclusive serial owner, on a dedicated robot supervision thread.

    The stop call is unconditional; firmware watchdog is still required to handle
    host failure / cable disconnection. Do not run floor tests before calibration.
    """
    def __init__(self, motor_bridge, calibration, settings=None, *, clock=time.monotonic, sleep=time.sleep):
        self.motors = motor_bridge
        self.odo = EncoderOdometry(calibration)
        self.s = settings or TurnSettings()
        self.clock = clock
        self.sleep = sleep

    def run(self, action, *, abort=None, log=None):
        if action not in ('LEFT', 'RIGHT', 'UTURN'):
            raise ValueError('Only LEFT/RIGHT/UTURN; STRAIGHT must use line follower')
        target = {'LEFT': 90.0, 'RIGHT': -90.0, 'UTURN': 180.0}[action]
        direction = 1 if target > 0 else -1
        start = self.clock()
        last_progress_time = start
        last_progress_deg = 0.0
        stage = None
        done = False
        observed = 0.0
        error = None
        history = []
        try:
            self.motors.stop()
            self.odo.reset(self.motors.get_encoder_counts())
            while True:
                if abort is not None and abort():
                    raise TurnFault('External abort')
                now = self.clock()
                if now - start > self.s.max_turn_s:
                    raise TurnFault('Turn timeout')
                sample = self.odo.update(self.motors.get_encoder_counts())
                observed = degrees(self.odo.total_yaw_rad) * direction
                delta = degrees(sample.yaw_rad) * direction
                if sample.disagreement_m > self.s.max_encoder_disagreement_m:
                    raise TurnFault('Front/rear encoder disagreement')
                if abs(delta) > self.s.max_sample_jump_deg:
                    raise TurnFault('Encoder angle jump')
                if observed < -4:
                    raise TurnFault('Wrong turn direction')
                if observed - last_progress_deg >= self.s.min_motion_deg:
                    last_progress_deg = observed
                    last_progress_time = now
                remaining = abs(target) - observed
                history.append((round(now-start,3),round(observed,2),round(remaining,2)))
                if log: log(history[-1])
                if remaining <= self.s.stop_early_deg:
                    done = True
                    break
                if now - last_progress_time > self.s.no_motion_s:
                    raise TurnFault('Stall: no encoder motion')
                pwm = self.s.slow_pwm if remaining <= self.s.slow_at_remaining_deg else self.s.fast_pwm
                if pwm != stage:
                    if direction > 0:
                        wheels = (pwm,-pwm,pwm,-pwm)
                    else:
                        wheels = (-pwm,pwm,-pwm,pwm)
                    self.motors.set_wheels(*wheels)
                    stage = pwm
                else:
                    # Refresh watchdog even if PWM unchanged; only one owner of serial link.
                    self.motors.set_wheels(*( (pwm,-pwm,pwm,-pwm) if direction > 0
                                             else (-pwm,pwm,-pwm,pwm) ))
                self.sleep(self.s.poll_s)
        except Exception as exc:
            error = exc
        finally:
            try:
                self.motors.stop()
            except Exception as stop_exc:
                if error is None: error = TurnFault(f'Cannot confirm stop: {stop_exc}')
        if error is not None:
            raise TurnFault(str(error)) from error
        return {'action':action, 'encoder_threshold_reached':done,
                'estimated_angle_deg':round(observed * direction,2),
                'line_reacquired':False, 'maneuver_completed':False,
                'samples':history}

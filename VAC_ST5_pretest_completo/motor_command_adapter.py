"""T.3 command interface, purposely decoupled from physical maneuver confirmation."""
from dataclasses import dataclass


@dataclass
class MotorCommandAdapter:
    bridge: object

    def supervised_test_command(self, navigation_request, *, pwm=60, duration_s=0.15):
        """Bench-only: call after receiving next_command(), never auto-confirm it.

        Physical action is only a short test pulse; `maneuver_result(True, True)`
        must come from later real controller + camera confirmations, not here.
        """
        if navigation_request.get('event') != 'MANEUVER_REQUEST':
            raise ValueError('Expected a T.3 MANEUVER_REQUEST')
        action = navigation_request['action']
        if action == 'ALIGN_REQUIRED':
            raise RuntimeError('Curve START-Y needs special alignment procedure')
        return self.bridge.supervised_pulse(action, pwm=pwm, duration_s=duration_s)

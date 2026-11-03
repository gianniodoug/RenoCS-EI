"""Conservative Raspberry Pi -> Arduino serial motor driver (ST5 VAC).

Bench-only pulse execution; never declares an intersection turn complete.
No physical actuation on import. Requires explicit hardware opt-in.
The supplied Arduino firmware DOES NOT implement a functional watchdog.
"""
import struct
import time
from dataclasses import dataclass


class MotorCommunicationError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotorConfig:
    port: str = '/dev/ttyACM0'
    baudrate: int = 115200
    pwm_limit: int = 100  # deliberate conservative software cap, not a proven safe speed
    max_pulse_s: float = 0.3
    timeout_s: float = 0.4


class MotorBridge:
    def __init__(self, config=None, *, allow_hardware=False, serial_factory=None, sleep=time.sleep):
        self.config = config or MotorConfig()
        self.allow_hardware = allow_hardware
        self._factory = serial_factory
        self._sleep = sleep
        self._port = None
        self._connected = False

    def connect(self):
        if not self.allow_hardware:
            raise PermissionError('Hardware disabled: set allow_hardware=True only for a supervised bench test')
        if self._port is not None:
            raise RuntimeError('Already connected')
        if self._factory is None:
            import serial  # optional: only on Raspberry Pi / physical test
            self._factory = serial.Serial
        self._port = self._factory(port=self.config.port, baudrate=self.config.baudrate,
                                   timeout=self.config.timeout_s, write_timeout=self.config.timeout_s)
        try:
            self._sleep(2.0)  # Arduino resets on port open
            self._port.reset_input_buffer()
            # A12 = feedback 1 (OK/OB), binary args + binary telemetry; avoid I0:
            # in supplied firmware PROTECT_IR_code() is commented/no acknowledgement.
            self._port.write(b'A12')
            line = self._port.readline().strip()
            if line != b'OK':
                raise MotorCommunicationError(f'Connection rejected: {line!r}')
            self._connected = True
            self.stop()
            return True
        except Exception:
            self.close()
            raise

    def _send(self, pwms):
        if not self._connected:
            raise MotorCommunicationError('Not connected')
        if len(pwms) != 4 or any(type(x) is not int or abs(x) > self.config.pwm_limit for x in pwms):
            raise ValueError('Four signed integer PWM values within configured limit required')
        # C + 4 x little endian signed int16. M1 FR, M2 FL, M3 RR, M4 RL.
        self._port.write(b'C' + struct.pack('<4h', *pwms))
        ack = self._port.readline().strip()
        if ack != b'OK':
            raise MotorCommunicationError(f'Command not acknowledged: {ack!r}')

    def get_encoder_counts(self):
        """Read 4 cumulative signed 32-bit encoders in FR, FL, RR, RL order.

        Arduino `N` returns front pair, `n` returns rear pair in binary mode A12.
        This call must not race another reader on the same serial port.
        """
        if not self._connected:
            raise MotorCommunicationError('Not connected')
        try:
            values = []
            for command in (b'N', b'n'):
                self._port.write(command)
                raw = self._port.read(8)
                if len(raw) != 8:
                    raise MotorCommunicationError('Encoder reply incomplete')
                values.extend(struct.unpack('<2i', raw))
            return tuple(values)
        except Exception:
            self._best_effort_stop()
            raise

    def set_wheels(self, fr, fl, rr, rl):
        try:
            self._send((fr, fl, rr, rl))
        except Exception:
            self._best_effort_stop()
            raise

    def stop(self):
        self.set_wheels(0, 0, 0, 0)

    def _best_effort_stop(self):
        if self._port is not None:
            try:
                self._port.write(b'C' + struct.pack('<4h', 0, 0, 0, 0))
            except Exception:
                pass  # if USB disconnects, only a firmware/hardware watchdog can stop motors

    def supervised_pulse(self, action, pwm=60, duration_s=0.15):
        """One bounded motor test pulse, NOT a 90-degree/180-degree manoeuvre.

        STOP is attempted in finally even on exception. Physical action requires
        wheels lifted, nearby operator, local emergency stop and verified direction.
        """
        if action not in ('LEFT', 'RIGHT', 'STRAIGHT', 'BACK', 'UTURN'):
            raise ValueError('Unsupported action')
        if type(pwm) is not int or not 0 < pwm <= self.config.pwm_limit:
            raise ValueError('PWM must be a positive integer below cap')
        if not isinstance(duration_s, (int, float)) or not 0 < duration_s <= self.config.max_pulse_s:
            raise ValueError('Duration exceeds bounded test interval')
        # Matches supplied arduino.py sign conventions; actual directions require verification.
        outputs = {
            'LEFT': (pwm, -pwm, pwm, -pwm),
            'RIGHT': (-pwm, pwm, -pwm, pwm),
            'STRAIGHT': (pwm, pwm, pwm, pwm),
            'BACK': (-pwm, -pwm, -pwm, -pwm),
            'UTURN': (pwm, -pwm, pwm, -pwm),
        }
        try:
            self.set_wheels(*outputs[action])
            self._sleep(duration_s)
        finally:
            self._best_effort_stop()
        # No physical turn/line alignment confirmation is implied.
        return {'action': action, 'pulse_sent': True, 'maneuver_completed': False,
                'line_reacquired': False}

    def close(self):
        if self._port is not None:
            self._best_effort_stop()
            try:
                self._port.close()
            finally:
                self._port = None
                self._connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.close()

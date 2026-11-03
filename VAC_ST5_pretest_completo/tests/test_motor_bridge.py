import unittest
from motor_bridge import MotorBridge, MotorConfig, MotorCommunicationError
from motor_command_adapter import MotorCommandAdapter


class FakeSerial:
    def __init__(self, **kwargs):
        self.writes = []
        self.closed = False
        self.reply = b'OK\n'

    def write(self, buf):
        self.writes.append(buf)

    def readline(self):
        return self.reply

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.bridge = MotorBridge(config=MotorConfig(), allow_hardware=True,
                                  serial_factory=lambda **kwargs: self.fake, sleep=lambda t: None)
        self.bridge.connect()

    def tearDown(self):
        self.bridge.close()

    def test_handshake_without_i0(self):
        self.assertEqual(self.fake.writes[0], b'A12')
        self.assertNotIn(b'I0', self.fake.writes)

    def test_turn_left_command_frame(self):
        result = self.bridge.supervised_pulse('LEFT', pwm=50, duration_s=.1)
        self.assertIn(b'C\x32\x00\xce\xff\x32\x00\xce\xff', self.fake.writes)
        self.assertFalse(result['maneuver_completed'])
        self.assertEqual(self.fake.writes[-1], b'C' + b'\x00'*8)

    def test_invalid_pwm_does_not_move(self):
        with self.assertRaises(ValueError):
            self.bridge.supervised_pulse('RIGHT', pwm=200)

    def test_adapter_does_not_confirm_navigation(self):
        a = MotorCommandAdapter(self.bridge)
        r = a.supervised_test_command({'event':'MANEUVER_REQUEST','action':'RIGHT'})
        self.assertFalse(r['line_reacquired'])
        with self.assertRaises(RuntimeError):
            a.supervised_test_command({'event':'MANEUVER_REQUEST','action':'ALIGN_REQUIRED'})

    def test_permission_required(self):
        with self.assertRaises(PermissionError):
            MotorBridge(serial_factory=lambda **kwargs: self.fake).connect()

    def test_ack_failure_still_attempts_stop(self):
        self.fake.reply = b'OB\n'
        with self.assertRaises(MotorCommunicationError):
            self.bridge.supervised_pulse('LEFT')
        self.assertEqual(self.fake.writes[-1], b'C' + b'\x00'*8)


if __name__ == '__main__':
    unittest.main()

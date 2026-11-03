import math
import struct
import unittest
from encoder_odometry import (EncoderCalibration, EncoderOdometry,
    SegmentProgressEstimator, EncoderTurnMonitor)
from motor_bridge import MotorBridge, MotorConfig, MotorCommunicationError

CAL = EncoderCalibration(ticks_per_wheel_rev=1000, wheel_diameter_m=1/math.pi,
                         forward_signs=(1,1,1,1), effective_track_width_m=1)


class TestEncoders(unittest.TestCase):
    def test_forward_distance(self):
        o = EncoderOdometry(CAL)
        o.reset((0,0,0,0))
        s = o.update((250,250,250,250))
        self.assertAlmostEqual(s.distance_m, .25)
        self.assertAlmostEqual(s.yaw_rad, 0)

    def test_backward(self):
        o = EncoderOdometry(CAL)
        o.reset((0,0,0,0))
        self.assertAlmostEqual(o.update((-100,-100,-100,-100)).distance_m, -.1)

    def test_signed_encoder_configuration(self):
        cal = EncoderCalibration(1000, 1/math.pi, (1,-1,1,-1), 1)
        o = EncoderOdometry(cal)
        o.reset((0,0,0,0))
        self.assertAlmostEqual(o.update((100,-100,100,-100)).distance_m, .1)

    def test_overflow(self):
        o=EncoderOdometry(CAL)
        o.reset((2147483646,)*4)
        self.assertAlmostEqual(o.update((-2147483648,)*4).distance_m, .002)

    def test_progress(self):
        s=SegmentProgressEstimator(CAL, 2.0)
        s.reset((0,)*4)
        self.assertAlmostEqual(s.update((500,)*4)['progress'], .25)

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):
            EncoderCalibration(0, .1, (1,1,1,1), .2)

    def test_turn_left_and_line_still_unverified(self):
        m=EncoderTurnMonitor(CAL)
        m.begin((0,0,0,0), 'LEFT')
        d=round((math.pi/2)*500)
        result=m.update((d,-d,d,-d))
        self.assertEqual(result['status'], 'ANGLE_REACHED')
        self.assertFalse(result['line_reacquired'])
        self.assertFalse(result['physical_turn_confirmed'])

    def test_turn_right(self):
        m=EncoderTurnMonitor(CAL)
        m.begin((0,)*4,'RIGHT')
        d=round((math.pi/2)*500)
        self.assertEqual(m.update((-d,d,-d,d))['status'], 'ANGLE_REACHED')

    def test_wrong_turn_direction(self):
        m=EncoderTurnMonitor(CAL)
        m.begin((0,)*4,'LEFT')
        self.assertEqual(m.update((-200,200,-200,200))['status'], 'FAULT')

    def test_encoder_disagreement(self):
        m=EncoderTurnMonitor(CAL)
        m.begin((0,)*4,'LEFT')
        self.assertEqual(m.update((300,0,0,0))['status'], 'FAULT')


class EncoderSerialMock:
    def __init__(self, **kwargs):
        self.writes=[]
        self.replies=[struct.pack('<2i',12,-13),struct.pack('<2i',14,-15)]
    def write(self, raw): self.writes.append(raw)
    def readline(self): return b'OK\n'
    def reset_input_buffer(self): pass
    def read(self,n): return self.replies.pop(0)
    def close(self): pass


class TestBridgeEncoders(unittest.TestCase):
    def test_reads_front_and_rear(self):
        serial=EncoderSerialMock()
        bridge=MotorBridge(MotorConfig(),allow_hardware=True,
                           serial_factory=lambda **kwargs: serial,sleep=lambda x: None)
        with bridge:
            self.assertEqual(bridge.get_encoder_counts(),(12,-13,14,-15))
        self.assertIn(b'N',serial.writes)
        self.assertIn(b'n',serial.writes)

    def test_incomplete_read_stops(self):
        serial=EncoderSerialMock()
        serial.replies=[b'\0']
        bridge=MotorBridge(MotorConfig(),allow_hardware=True,
                           serial_factory=lambda **kwargs: serial,sleep=lambda x: None)
        with bridge:
            with self.assertRaises(MotorCommunicationError):
                bridge.get_encoder_counts()
            self.assertEqual(serial.writes[-1],b'C'+b'\0'*8)


if __name__=='__main__':
    unittest.main()

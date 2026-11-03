import unittest
from math import pi
from encoder_odometry import EncoderCalibration
from closed_loop_turn import ClosedLoopTurn,TurnSettings,TurnFault
from calibration_store import load_calibration
from pathlib import Path

class Clock:
    def __init__(self):self.t=0
    def clock(self):return self.t
    def sleep(self,dt):self.t+=dt

class FakeMotor:
    def __init__(self,step=2,stuck=False,fail=False):
        self.counts=[0,0,0,0]
        self.cmd=(0,0,0,0)
        self.step=step
        self.stuck=stuck
        self.fail=fail
        self.stop_count=0
    def stop(self):
        self.cmd=(0,0,0,0);self.stop_count+=1
    def set_wheels(self,*v):self.cmd=v
    def get_encoder_counts(self):
        if self.fail:raise IOError('lost serial')
        if not self.stuck:
            for i,x in enumerate(self.cmd):
                self.counts[i]+= (self.step if x>0 else -self.step if x<0 else 0)
        return tuple(self.counts)

# Exactly 0.01 m/tick; effective width 0.2m -> 0.1rad per sample when step=1.
CAL=EncoderCalibration(ticks_per_wheel_rev=pi/.01,wheel_diameter_m=1,
                       forward_signs=(1,1,1,1),effective_track_width_m=.2)

class TurnTests(unittest.TestCase):
    def make(self, motor, **kw):
        c=Clock()
        s=TurnSettings(poll_s=.05,no_motion_s=.4,max_turn_s=4, max_sample_jump_deg=30,
                       fast_pwm=60, slow_pwm=40, **kw)
        return ClosedLoopTurn(motor,CAL,s,clock=c.clock,sleep=c.sleep)
    def test_left_stops_with_threshold(self):
        m=FakeMotor(step=1);r=self.make(m).run('LEFT')
        self.assertTrue(r['encoder_threshold_reached']);self.assertFalse(r['line_reacquired'])
        self.assertEqual(m.cmd,(0,0,0,0));self.assertGreaterEqual(m.stop_count,2)
    def test_right_and_uturn(self):
        for a in ('RIGHT','UTURN'):
            m=FakeMotor(step=1)
            r=self.make(m).run(a)
            self.assertTrue(r['encoder_threshold_reached']);self.assertEqual(m.cmd,(0,0,0,0))
    def test_stall_stops(self):
        m=FakeMotor(stuck=True)
        with self.assertRaisesRegex(TurnFault,'Stall'):self.make(m).run('LEFT')
        self.assertEqual(m.cmd,(0,0,0,0))
    def test_serial_failure_stops(self):
        m=FakeMotor(fail=True)
        with self.assertRaises(TurnFault):self.make(m).run('LEFT')
        self.assertEqual(m.cmd,(0,0,0,0))
    def test_bad_settings(self):
        with self.assertRaises(ValueError):TurnSettings(stop_early_deg=25)
    def test_missing_calibration_rejected(self):
        with self.assertRaises(ValueError):load_calibration(Path(__file__).parents[1]/'calibration_TEMPLATE.json')

if __name__ == '__main__':unittest.main()

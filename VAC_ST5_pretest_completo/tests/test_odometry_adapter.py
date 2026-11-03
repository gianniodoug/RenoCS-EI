import math
import unittest
from grid_map import GridMap
from localization import RobotLocalizer
from navigation_controller import NavigationController
from odometry_adapter import NavigationOdometryAdapter
from encoder_odometry import EncoderCalibration

class TestAdapter(unittest.TestCase):
    def test_measured_distance_updates_localizer(self):
        g=GridMap()
        g.set_length('A','B',2.)
        loc=RobotLocalizer(g,current_node='A')
        loc.begin_segment('B') # emulate line already confirmed by T.3
        nav=NavigationController(g,loc,phase='FOLLOWING')
        nav.plan={'detailed_steps':[{'segment':('A','B'),'end_progress':1.}]}
        nav.step_index=0
        c=EncoderCalibration(1000,1/math.pi,(1,1,1,1),1.)
        adapter=NavigationOdometryAdapter(nav,c)
        adapter.begin_following((0,0,0,0))
        adapter.update((500,500,500,500))
        self.assertAlmostEqual(loc.progress,.25)
    def test_no_length_rejected(self):
        g=GridMap()
        loc=RobotLocalizer(g,current_node='A')
        loc.begin_segment('B')
        nav=NavigationController(g,loc,phase='FOLLOWING')
        c=EncoderCalibration(1000,1/math.pi,(1,1,1,1),1.)
        with self.assertRaises(RuntimeError):
            NavigationOdometryAdapter(nav,c).begin_following((0,0,0,0))
if __name__=='__main__': unittest.main()

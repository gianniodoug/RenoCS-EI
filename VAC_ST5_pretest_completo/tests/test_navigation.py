import unittest
from grid_map import GridMap
from localization import RobotLocalizer
from navigation_controller import NavigationController

class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap()
        self.loc = RobotLocalizer(self.grid, current_node='B', orientation='E')
        self.ctl = NavigationController(self.grid, self.loc)

    def test_turn_commands_and_confirmations(self):
        self.ctl.load_mission({'type':'node','node':'G'})
        while self.ctl.phase != 'COMPLETED':
            command = self.ctl.next_command()
            self.assertIn(command['action'], ('LEFT','RIGHT','STRAIGHT','UTURN','ALIGN_REQUIRED'))
            self.assertEqual(self.ctl.maneuver_result(True, True)['event'], 'FOLLOW_LINE_REQUEST')
            step = self.ctl.plan['detailed_steps'][self.ctl.step_index]
            self.ctl.progress_update(1.0)
            self.ctl.confirmed_intersection(step['segment'][1])
        self.assertEqual(self.loc.current_node, 'G')

    def test_not_confirmed_before_alignment(self):
        self.ctl.load_mission({'type':'node','node':'C'})
        with self.assertRaises(RuntimeError): self.ctl.confirmed_intersection('C')
        self.ctl.next_command()
        with self.assertRaises(RuntimeError): self.ctl.confirmed_intersection('C')
        self.assertEqual(self.loc.current_node, 'B')

    def test_failure_stops_without_position_jump(self):
        self.ctl.load_mission({'type':'node','node':'C'})
        self.ctl.next_command()
        self.assertEqual(self.ctl.maneuver_result(True, False)['event'], 'STOP_REQUEST')
        self.assertEqual(self.loc.current_node, 'B')
        self.assertIsNone(self.loc.from_node)

    def test_wrong_intersection_stops(self):
        self.ctl.load_mission({'type':'node','node':'C'})
        self.ctl.next_command(); self.ctl.maneuver_result(True,True)
        self.assertEqual(self.ctl.confirmed_intersection('D')['event'], 'STOP_REQUEST')
        self.assertEqual(self.loc.current_node, 'B')

    def test_partial_delivery(self):
        self.ctl.load_mission({'type':'edge','edge':('B','C'),'progress':.75})
        self.ctl.next_command(); self.ctl.maneuver_result(True,True)
        self.ctl.progress_update(.75)
        self.assertEqual(self.ctl.reached_partial_target(True)['event'], 'MISSION_COMPLETED')
        self.assertAlmostEqual(self.loc.progress,.75)

    def test_early_partial_stop_rejected(self):
        self.ctl.load_mission({'type':'edge','edge':('B','C'),'progress':.75})
        self.ctl.next_command(); self.ctl.maneuver_result(True,True)
        self.ctl.progress_update(.3)
        self.assertEqual(self.ctl.reached_partial_target(True)['event'],'STOP_REQUEST')

    def test_no_mid_edge_mission_load(self):
        self.loc.begin_segment('C')
        with self.assertRaises(RuntimeError): self.ctl.load_mission({'type':'node','node':'G'})

    def test_uturn_from_heading(self):
        self.loc.orientation = 'W'
        self.ctl.load_mission({'type':'node','node':'C'})
        self.assertEqual(self.ctl.next_command()['action'],'UTURN')

if __name__ == '__main__': unittest.main()

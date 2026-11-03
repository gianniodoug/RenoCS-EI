import unittest
from grid_map import GridMap, BASE
from localization import RobotLocalizer
from path_planning import plan_mission_from_state, handle_obstacle_and_reroute, plan_return_to_base, turn_command

class IntegratedTests(unittest.TestCase):
    def setUp(self):
        self.g = GridMap()
        self.loc = RobotLocalizer(self.g)

    def test_start_to_grid_and_return(self):
        p = plan_mission_from_state(self.g, self.loc.snapshot(), {'type':'node','node':'G'})
        self.assertTrue(p['feasible'])
        self.assertEqual(p['path'][0], BASE)
        self.assertIn('ALIGN_REQUIRED', p['commands'])
        self.assertEqual(plan_return_to_base(self.g, p['expected_end_state'])['path'][-1], BASE)

    def test_same_edge_forward_and_reverse(self):
        self.loc.current_node = 'B'
        self.loc.begin_segment('C')
        self.loc.update_progress(.3)
        p = plan_mission_from_state(self.g, self.loc.snapshot(),
            {'type':'edge','edge':('B','C'),'progress':.7})
        self.assertTrue(p['feasible'])
        self.assertAlmostEqual(p['total_cost'], .4)
        self.assertEqual(p['commands'], ['STRAIGHT'])
        p2 = plan_mission_from_state(self.g, self.loc.snapshot(),
            {'type':'edge','edge':('B','C'),'progress':.1})
        self.assertTrue(p2['feasible'])
        self.assertAlmostEqual(p2['total_cost'], .2)
        self.assertEqual(p2['commands'], ['UTURN'])

    def test_same_position(self):
        self.loc.current_node = 'B'
        self.loc.begin_segment('C')
        self.loc.update_progress(.3)
        p = plan_mission_from_state(self.g, self.loc.snapshot(),
            {'type':'edge','edge':('C','B'),'progress':.7})
        self.assertTrue(p['feasible'])
        self.assertAlmostEqual(p['total_cost'], 0.)
        self.assertEqual(p['commands'], [])

    def test_obstacle_recovery_is_not_teleport(self):
        self.loc.current_node = 'B'
        self.loc.begin_segment('C')
        self.loc.update_progress(.4)
        before = self.loc.snapshot().copy()
        r = handle_obstacle_and_reroute(self.g, before, {'type':'node','node':'D'})
        self.assertTrue(r['recovery_required'])
        self.assertFalse(r['may_execute_reroute_now'])
        self.assertEqual(self.loc.snapshot(), before)
        self.assertTrue(r['reroute_after_confirmed_arrival']['feasible'])
        self.assertFalse(plan_mission_from_state(self.g, before, {'type':'node','node':'D'},
            r['blocked_edges'])['feasible'])

    def test_blocked_target(self):
        p = plan_mission_from_state(self.g, self.loc.snapshot(),
            {'type':'edge','edge':('B','C'),'progress':.5}, [('B','C')])
        self.assertFalse(p['feasible'])

    def test_obstacle_alternate(self):
        self.loc.current_node = 'B'
        p = plan_mission_from_state(self.g, self.loc.snapshot(),
            {'type':'node','node':'C'}, [('B','C')])
        self.assertTrue(p['feasible'])
        self.assertNotIn(('B','C'), [s['segment'] for s in p['detailed_steps']])

    def test_measured_cost(self):
        for a in self.g.adjacency:
            for b in self.g.adjacency[a]:
                self.g.set_length(a,b,2.0)
        p = plan_mission_from_state(self.g, self.loc.snapshot(), {'type':'node','node':'Y'})
        self.assertEqual(p['cost_unit'], 'm')
        self.assertEqual(p['total_cost'], 2.)

    def test_turns(self):
        self.assertEqual(turn_command('E','N'),'LEFT')
        self.assertEqual(turn_command('N','E'),'RIGHT')
        self.assertEqual(turn_command(None,'W'),'ALIGN_REQUIRED')

if __name__ == '__main__': unittest.main()

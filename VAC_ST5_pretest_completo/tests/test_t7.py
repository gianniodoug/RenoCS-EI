import unittest
from grid_map import GridMap, BASE, edge_key
from localization import RobotLocalizer

class T7Tests(unittest.TestCase):
    def setUp(self):
        self.g = GridMap()

    def test_topology(self):
        self.assertEqual(len(self.g.coordinates), 26)
        self.assertEqual(sum(map(len, self.g.adjacency.values())) // 2, 41)
        self.assertEqual(self.g.adjacency[BASE], {'Y'})
        self.assertEqual(self.g.direction('A', 'B'), 'E')
        self.assertIsNone(self.g.direction(BASE, 'Y'))

    def test_depart_arrive(self):
        loc = RobotLocalizer(self.g)
        loc.begin_segment('Y')
        self.assertIsNone(loc.orientation)
        loc.confirm_arrival('Y')
        loc.begin_segment('X')
        self.assertEqual(loc.orientation, 'W')
        loc.confirm_arrival('X')
        self.assertEqual(loc.current_node, 'X')

    def test_reverse_midway(self):
        loc = RobotLocalizer(self.g, current_node='B')
        loc.begin_segment('C')
        loc.update_progress(.4)
        loc.reverse_direction()
        self.assertEqual((loc.from_node, loc.to_node), ('C', 'B'))
        self.assertAlmostEqual(loc.progress, .6)
        self.assertEqual(loc.orientation, 'W')
        loc.confirm_arrival('B')
        self.assertEqual(loc.current_node, 'B')

    def test_obstacle(self):
        self.g.set_length('B', 'C', 1.0)
        loc = RobotLocalizer(self.g, current_node='B')
        loc.begin_segment('C')
        loc.update_progress(.4)
        self.assertEqual(loc.obstacle_position(.3)['fraction'], .7)
        self.assertIsNone(loc.obstacle_position(.8))

    def test_no_metre_assumption(self):
        loc = RobotLocalizer(self.g, current_node='B')
        loc.begin_segment('C')
        with self.assertRaises(RuntimeError):
            loc.update_distance_m(.2)

if __name__ == '__main__':
    unittest.main()

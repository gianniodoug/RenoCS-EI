"""Software-only integration demo: events are simulated, no motors or sensors."""
from grid_map import GridMap, BASE
from localization import RobotLocalizer
from path_planning import plan_mission_from_state, handle_obstacle_and_reroute, plan_return_to_base


def main():
    grid = GridMap()
    robot = RobotLocalizer(grid)
    print('Griglia:', len(grid.coordinates), 'nodi; base:', BASE)
    plan = plan_mission_from_state(grid, robot.snapshot(), {'type': 'node', 'node': 'G'})
    print('Missione START -> G:', plan['path'], plan['commands'], plan['total_cost'], plan['cost_unit'])
    robot.begin_segment('Y')
    robot.confirm_arrival('Y')  # SIMULATED confirmation only
    robot.begin_segment('X')
    robot.update_progress(.4)  # SIMULATED odometry only
    print('Posizione T7:', robot.snapshot())
    print('Nuova missione (75% X->W):', plan_mission_from_state(
        grid, robot.snapshot(), {'type': 'edge', 'edge': ('X', 'W'), 'progress': .75})['path'])
    print('Ostacolo:', handle_obstacle_and_reroute(
        grid, robot.snapshot(), {'type': 'node', 'node': 'G'})['recovery_step'])
    robot.reverse_direction()  # only once real UTURN is completed
    robot.confirm_arrival('Y')  # only once actual arrival has been verified
    print('Stato dopo recupero simulato:', robot.snapshot())
    print('Rientro:', plan_return_to_base(grid, robot.snapshot())['path'])

if __name__ == '__main__':
    main()

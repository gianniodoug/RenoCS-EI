"""Entirely simulated camera, manoeuvre and line-follow acknowledgements."""
from grid_map import GridMap, BASE
from localization import RobotLocalizer
from navigation_controller import NavigationController


def run_demo():
    grid = GridMap()
    ctl = NavigationController(grid, RobotLocalizer(grid))
    print(ctl.load_mission({'type': 'node', 'node': 'W'}))
    while ctl.phase != 'COMPLETED':
        cmd = ctl.next_command()
        print('Send to future motor controller:', cmd)
        print(ctl.maneuver_result(success=True, line_reacquired=True))  # simulated ONLY
        step = ctl.plan['detailed_steps'][ctl.step_index]
        print(ctl.progress_update(step['end_progress']))  # simulated ONLY
        if step['end_progress'] < 1:
            print(ctl.reached_partial_target(stopped=True))
        else:
            print(ctl.confirmed_intersection(step['segment'][1]))  # simulated ONLY
    print('Final:', ctl.snapshot())


if __name__ == '__main__':
    run_demo()

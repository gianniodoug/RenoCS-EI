"""DRY RUN ONLY by default. Real motion explicitly gated.

Usage:
  python test_turn_controller.py --simulate
  python test_turn_controller.py --hardware --bench --calibration calibration.json --action LEFT

Requires watchdog firmware and a tested hard power cutoff. NEVER run on floor
without prior validation, guards against unexpected movement and supervisor.
"""
import argparse
from pathlib import Path
from calibration_store import load_calibration
from closed_loop_turn import ClosedLoopTurn
from motor_bridge import MotorBridge


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--simulate',action='store_true')
    p.add_argument('--hardware',action='store_true')
    p.add_argument('--bench',action='store_true')
    p.add_argument('--calibration')
    p.add_argument('--action',choices=['LEFT','RIGHT','UTURN'],default='LEFT')
    p.add_argument('--port',default='/dev/ttyACM0')
    args=p.parse_args()
    if args.simulate and not args.hardware:
        print('No hardware used. Run: python simulation_odometry.py')
        return
    if not args.hardware or not args.bench:
        p.error('Real motors require --hardware --bench, tested watchdog, lifted wheels, operator and physical cutoff')
    if args.calibration is None:
        p.error('Provide measured calibration using --calibration')
    cal=load_calibration(args.calibration)
    print('BENCH ONLY: encoder yaw is NOT valid for free-spinning wheels; do NOT treat this as physical-angle calibration.')
    print('This controller will command real motors. Ensure robot is restrained, wheel rotation directions verified.')
    if input('Type BENCH to acknowledge: ') != 'BENCH':
        print('Cancelled');return
    from motor_bridge import MotorConfig
    with MotorBridge(config=MotorConfig(port=args.port), allow_hardware=True) as motors:
        result=ClosedLoopTurn(motors,cal).run(args.action)
        print(result)
        print('Check physical stop and independently verify orientation; no line confirmation is assumed.')

if __name__=='__main__': main()

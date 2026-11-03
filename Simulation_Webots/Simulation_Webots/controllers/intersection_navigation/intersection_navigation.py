"""Line following plus validated X/T stop requests; restart to release a stop.

Select controller 'intersection_navigation' on the existing Cherokey robot.
PiCam is BGRA8; the detector/follower receive unflipped, unannotated BGR8.
Motor order is left-front, right-front, left-rear, right-rear (2,1,4,3).
Positive targets drive forward. Only this controller writes wheel velocities.
"""
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from Basique_Navigation.suivi_ligne import LineFollower, MOTOR_NAMES, TIME_STEP
from intersections.intersection_pipeline import IntersectionPipeline


def run(robot):
    camera = robot.getDevice("PiCam")
    camera.enable(TIME_STEP)
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    follower = LineFollower()
    pipeline = IntersectionPipeline(run_id="webots-intersection-navigation")
    seq = 0
    try:
        while robot.step(TIME_STEP) != -1:
            seq += 1
            t_ns = round(robot.getTime() * 1_000_000_000)
            image = None
            try:
                raw = camera.getImage()
                if raw:
                    bgra = np.frombuffer(raw, np.uint8).reshape(
                        camera.getHeight(), camera.getWidth(), 4)
                    image = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            except (ValueError, RuntimeError, cv2.error):
                pass  # The pipeline latches a fault stop for the missing frame.
            result = pipeline.process_bgr(image, t_ns=t_ns, seq=seq, now_ns=t_ns)
            targets = (0.0,) * 4
            if not result["stop_requested"]:
                left, right, _ = follower.update(image)
                targets = (left, right, left, right)
            if result["stop_action"]:
                # Preserve the original follower's motor response during movement.
                # Apply the measured simulation braking limit only on a stop.
                for motor in motors:
                    motor.setAcceleration(3.0)
            for motor, target in zip(motors, targets):
                motor.setVelocity(target)
            if result["stop_action"]:
                print(f"INTERSECTION_STOP {t_ns} {result['stop_reason']} "
                      f"{result['observation']}", flush=True)
    finally:
        for motor in motors:
            motor.setVelocity(0.0)


if __name__ == "__main__":
    from controller import Robot
    run(Robot())

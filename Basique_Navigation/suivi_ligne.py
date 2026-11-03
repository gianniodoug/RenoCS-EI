"""Original centroid/PD line following, reusable without starting Webots."""
import numpy as np
import cv2

TIME_STEP = 64
BASE_SPEED = 4.0
KP = 0.04
KD = 0.015
MAX_SPEED = 10.0
MOTOR_NAMES = ("Motor2", "Motor1", "Motor4", "Motor3")


class LineFollower:
    """Same bottom-30% threshold, centroid, gains and wheel limits as the original.

    update accepts a uint8 BGR frame and returns left/right rad/s plus the
    annotated binary ROI. Missing line => zero speeds, retaining previous error.
    This class owns no Webots devices and sends no motor commands.
    """
    def __init__(self):
        self.previous_error = 0

    def update(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        roi = gray[int(h * 0.70):h, :]
        _, binary = cv2.threshold(roi, 80, 255, cv2.THRESH_BINARY)
        moments = cv2.moments(binary)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            error = cx - w / 2
            derivative = error - self.previous_error
            correction = KP * error + KD * derivative
            self.previous_error = error
            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction
            left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
            right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
            cv2.circle(binary, (cx, binary.shape[0] // 2), 5, 128, -1)
        else:
            left_speed = right_speed = 0
        return left_speed, right_speed, binary


def main():
    from controller import Robot
    robot = Robot()
    camera = robot.getDevice("PiCam")
    camera.enable(TIME_STEP)
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    follower = LineFollower()
    while robot.step(TIME_STEP) != -1:
        image = np.frombuffer(camera.getImage(), np.uint8).reshape(
            camera.getHeight(), camera.getWidth(), 4)
        left, right, binary = follower.update(cv2.cvtColor(image, cv2.COLOR_BGRA2BGR))
        for motor, speed in zip(motors, (left, right, left, right)):
            motor.setVelocity(speed)
        cv2.imshow("Camera", image)
        cv2.imshow("Detection ligne", binary)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()

import numpy as np
import cv2
from controller import Robot

TIME_STEP = 64
robot = Robot()
camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)

motors = []
motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]

for name in motors_names:
    motor = robot.getDevice(name)
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)
    motors.append(motor)

BASE_SPEED = 4.0
KP = 0.04
KD = 0.015
previous_error = 0
MAX_SPEED = 10.0

def set_motors(left_speed, right_speed):
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    motors[0].setVelocity(left_speed)
    motors[2].setVelocity(left_speed)
    motors[1].setVelocity(right_speed)
    motors[3].setVelocity(right_speed)

while robot.step(TIME_STEP) != -1:

    cameraData = camera.getImage()
    image = np.frombuffer(
        cameraData, np.uint8
    ).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

    gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    h, w = gray.shape

    roi = gray[int(h * 0.7):h, :]

    _, binary = cv2.threshold(
        roi, 80, 255, cv2.THRESH_BINARY
    )

    scan_positions = [
        int(binary.shape[0] * 0.9),
        int(binary.shape[0] * 0.75),
        int(binary.shape[0] * 0.6)
    ]

    centers = []

    for y in scan_positions:
        white_pixels = np.where(binary[y, :] > 0)[0]

        if len(white_pixels) > 0:
            groups = np.split(
                white_pixels,
                np.where(np.diff(white_pixels) > 1)[0] + 1
            )

            group_centers = [
                int((g[0] + g[-1]) / 2)
                for g in groups if len(g) > 0
            ]

            cx_row = min(
                group_centers,
                key=lambda x: abs(x - w / 2)
            )

            centers.append(cx_row)

    if len(centers) > 0:

        cx = int(np.mean(centers))

        error = cx - w / 2
        derivative = error - previous_error

        correction = KP * error + KD * derivative
        previous_error = error

        left_speed = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction

        set_motors(left_speed, right_speed)

        cv2.circle(
            binary,
            (cx, binary.shape[0] // 2),
            5, 128, -1
        )

    else:
        set_motors(0, 0)

    cv2.imshow("Camera", image)
    cv2.imshow("Detection ligne", binary)
    cv2.waitKey(1)
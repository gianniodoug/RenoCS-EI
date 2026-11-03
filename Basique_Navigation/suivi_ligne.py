import numpy as np
import cv2
import time

from picamera2 import Picamera2
from arduino import arduino


# -------------------------
# INITIALISATION CAMERA
# -------------------------

camera = Picamera2()
camera.configure(
    camera.create_preview_configuration(
        main={"format": "RGB888", "size": (320, 240)}
    )
)
camera.start()
time.sleep(2)


# -------------------------
# INITIALISATION ARDUINO
# -------------------------

robot = arduino('/dev/ttyACM0')

if not robot.connect():
    camera.stop()
    raise RuntimeError("Erreur de connexion Arduino")


# -------------------------
# PARAMETRES DU SUIVI
# -------------------------

BASE_SPEED = 100      # PWM (-255..255)
MAX_SPEED = 255

# The error is normalised: -1 = line at the left edge, +1 = right edge
KP = 110.0    # PWM per unit of error (= 0.35 per pixel at 640 px)
KI = 40.0     # PWM per (unit of error x second)
KD = 5.0      # PWM per (unit of error / second)
I_MAX = 60.0  # max PWM the integral term may add (anti-windup)

SHOW_DEBUG = True     # set to False during real runs (faster loop)

previous_error = 0.0
integral = 0.0
last_time = time.monotonic()

# -------------------------
# BOUCLE PRINCIPALE
# -------------------------

try:
    while True:

        # 1. Recuperation de l'image
        image = camera.capture_array()

        # 2. Conversion en niveaux de gris
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape

        # 3. Selection de la zone basse
        roi = gray[int(h * 0.7):h, :]

        # 4. Detection de la ligne blanche
        _, binary = cv2.threshold(
            roi, 80, 255, cv2.THRESH_BINARY
        )

        # 5. Positions des trois lignes de balayage
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

        # 6. Error and PID correction
        now = time.monotonic()
        dt = max(now - last_time, 1e-3)   # real time between frames
        last_time = now

        if len(centers) > 0:

            cx = int(np.mean(centers))

            error = (cx - w / 2) / (w / 2)     # normalised error, -1..+1

            # Integral: builds up the extra turn needed in the loops of the 8
            integral += error * dt
            integral = max(-I_MAX / KI, min(I_MAX / KI, integral))  # anti-windup

            derivative = (error - previous_error) / dt
            previous_error = error

            correction = KP * error + KI * integral + KD * derivative

            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

            left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
            right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

            # 7. Command the motors
            robot.SetMotors(left_speed, right_speed)

            if SHOW_DEBUG:
                print(f"err={error:+.2f} I={KI*integral:+.0f} "
                      f"corr={correction:+.0f} L={left_speed:.0f} R={right_speed:.0f}")
                cv2.circle(binary, (cx, binary.shape[0] // 2), 5, 128, -1)

        else:
            robot.Stop()
            integral = 0.0          # forget the old curve
            previous_error = 0.0

        # 8. Display (slow over VNC: disable it for real runs)
        if SHOW_DEBUG:
            cv2.imshow("Camera", image)
            cv2.imshow("Detection ligne", binary)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
except KeyboardInterrupt:
    pass

finally:
    robot.Stop()
    robot.disconnect()
    camera.stop()
    cv2.destroyAllWindows()
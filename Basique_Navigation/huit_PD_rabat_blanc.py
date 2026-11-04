
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
        main={"format": "RGB888", "size": (640, 480)}
    )
)
camera.start()
time.sleep(2)

# Désactivation des automatismes caméra
camera.set_controls({
    "AeEnable": False,
    "AwbEnable": False,
    "ExposureTime": 10000,
    "AnalogueGain": 1.0
})

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

BASE_SPEED = 100
KP = 0.04
KD = 0.015
previous_error = 0
MAX_SPEED = 255

SEARCH_SPEED = 25
MAX_SEARCH_TIME = 1.0
lost_since = None
last_error_sign = 1

WHITE_LOST_THRESHOLD = 70

# -------------------------
# BOUCLE PRINCIPALE
# -------------------------

try:
    while True:

        image = camera.capture_array()

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        h, w = gray.shape

        roi = gray[int(h * 0.7):h, :]

        _, binary = cv2.threshold(
            roi, 80, 255, cv2.THRESH_BINARY
        )

        # Détection d'une surface presque entièrement blanche
        white_percentage = (
            cv2.countNonZero(binary) / binary.size
        ) * 100

        white_screen = white_percentage >= WHITE_LOST_THRESHOLD

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

        # Suivi normal uniquement si l'image n'est pas trop blanche
        if len(centers) > 0 and not white_screen:

            cx = int(np.mean(centers))

            error = cx - w / 2
            derivative = error - previous_error

            correction = KP * error + KD * derivative
            previous_error = error

            if abs(error) > 2:
                last_error_sign = 1 if error > 0 else -1

            lost_since = None

            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction

            left_speed = int(max(
                -MAX_SPEED, min(MAX_SPEED, left_speed)
            ))

            right_speed = int(max(
                -MAX_SPEED, min(MAX_SPEED, right_speed)
            ))

            robot.SetMotors(left_speed, right_speed)

            cv2.circle(
                binary,
                (cx, binary.shape[0] // 2),
                5, 128, -1
            )

        else:

            if lost_since is None:
                lost_since = time.time()

            if time.time() - lost_since < MAX_SEARCH_TIME:

                robot.SetMotors(
                    -last_error_sign * SEARCH_SPEED,
                    last_error_sign * SEARCH_SPEED
                )

            else:
                robot.Stop()

        # Affichage
        cv2.putText(
            binary,
            f"Blanc : {white_percentage:.1f}%",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            128,
            2
        )

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

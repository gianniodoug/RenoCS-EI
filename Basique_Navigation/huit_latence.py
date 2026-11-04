
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

BASE_SPEED = 100
MIN_SPEED = 50
MAX_SPEED = 255

KP = 0.10
KD = 0.002

previous_error = 0
previous_time = time.perf_counter()

# Limitation de la correction
MAX_CORRECTION = 100

# Adaptation de la vitesse
SPEED_REDUCTION = 0.25

# Seuil de detection du blanc
THRESHOLD = 80

# Parametres de recherche de ligne
SEARCH_SPEED = 25
MAX_SEARCH_TIME = 1.0

lost_since = None
last_error_sign = 1


# -------------------------
# BOUCLE PRINCIPALE
# -------------------------

try:

    while True:

        loop_start = time.perf_counter()

        # 1. Recuperation de l'image

        image = camera.capture_array()

        # 2. Conversion en niveaux de gris

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        h, w = gray.shape

        # 3. Selection de la zone basse

        roi = gray[int(h * 0.7):h, :]

        # 4. Detection de la ligne blanche

        _, binary = cv2.threshold(
            roi,
            THRESHOLD,
            255,
            cv2.THRESH_BINARY
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

                # Selection du groupe le plus proche
                # du centre de l'image

                cx_row = min(
                    group_centers,
                    key=lambda x: abs(x - w / 2)
                )

                centers.append(cx_row)

        # 6. Calcul de l'erreur et correction PD

        if len(centers) > 0:

            # Moyenne ponderee des trois detections

            if len(centers) == 3:

                cx = int(
                    0.2 * centers[0] +
                    0.3 * centers[1] +
                    0.5 * centers[2]
                )

            else:

                cx = int(np.mean(centers))

            # Calcul de l'erreur

            error = cx - w / 2

            # Calcul du temps ecoule

            current_time = time.perf_counter()

            dt = current_time - previous_time

            dt = max(dt, 0.001)

            # Derivee temporelle

            derivative = (
                error - previous_error
            ) / dt

            previous_error = error
            previous_time = current_time

            # Correction PD

            correction = KP * error + KD * derivative

            # Limitation de la correction

            correction = max(
                -MAX_CORRECTION,
                min(MAX_CORRECTION, correction)
            )

            # Memorisation du dernier cote connu

            if abs(error) > 2:

                last_error_sign = (
                    1 if error > 0 else -1
                )

            # Ligne retrouvee

            lost_since = None

            # 7. Adaptation automatique de la vitesse

            base_speed_local = (
                BASE_SPEED -
                SPEED_REDUCTION * abs(error)
            )

            base_speed_local = max(
                MIN_SPEED,
                base_speed_local
            )

            # 8. Calcul des vitesses des moteurs

            left_speed = (
                base_speed_local + correction
            )

            right_speed = (
                base_speed_local - correction
            )

            # Limitation des commandes

            left_speed = int(max(
                -MAX_SPEED,
                min(MAX_SPEED, left_speed)
            ))

            right_speed = int(max(
                -MAX_SPEED,
                min(MAX_SPEED, right_speed)
            ))

            # 9. Commande des moteurs physiques

            robot.SetMotors(
                left_speed,
                right_speed
            )

            # Affichage du centre detecte

            cv2.circle(
                binary,
                (cx, binary.shape[0] // 2),
                5,
                128,
                -1
            )

        else:

            # 10. Aucun barycentre detecte : perte de ligne

            if lost_since is None:

                lost_since = time.time()

            if time.time() - lost_since < MAX_SEARCH_TIME:

                # Rotation vers le dernier cote connu

                robot.SetMotors(
                    -last_error_sign * SEARCH_SPEED,
                    last_error_sign * SEARCH_SPEED
                )

            else:

                robot.Stop()

        # 11. Affichage

        loop_time = time.perf_counter() - loop_start

        cv2.putText(
            image,
            "Loop: {:.1f} ms".format(loop_time * 1000),
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
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

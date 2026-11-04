import numpy as np
import cv2
import time

from picamera2 import Picamera2
from arduino import arduino

# Version fusionnee :
#  - base : huit_PD_anticipation_rabat.py (Alain)
#    -> orientation camera par STEERING_SIGN, zone d'analyse 50 %,
#       anticipation du virage, "rabat" quand la ligne est perdue
#  - image reduite a 320x240 (suivi_ligne.py, Ioannis)
#  - correction PD plus forte que la version d'Alain (erreur en pixels)
#  - correction : last_error_sign est mis a jour APRES le calcul de l'erreur


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

BASE_SPEED = 80

# Erreur en pixels (image de 320 px de large).
# Alain : KP 0.04 / KD 0.015 a 640 px  = 0.08 / 0.03 a 320 px.
# Ici environ 2.5x plus fort : 80 px d'erreur -> correction de 16 PWM.
KP = 0.2
KD = 0.075

previous_error = 0

MAX_SPEED = 200

# Signe inverse conserve des essais physiques (camera montee a l'envers)
STEERING_SIGN = -1

# Zone d'analyse : moitie basse de l'image
ROI_START = 0.50

# Position verticale utilisee pour anticiper le virage
LOOKAHEAD_RATIO = 0.45

# Parametres de rabat (recherche de la ligne perdue)
SEARCH_SPEED = 25        # a augmenter (50-60) si le robot ne tourne pas pendant la recherche
MAX_SEARCH_TIME = 1.0    # duree maximale de recherche en secondes
lost_since = None
last_error_sign = 1


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

        # 3. Selection de la zone d'analyse
        roi = gray[int(h * ROI_START):h, :]

        # 4. Detection de la ligne blanche
        _, binary = cv2.threshold(
            roi, 80, 255, cv2.THRESH_BINARY
        )

        # 5. Trois lignes de balayage
        scan_positions = [
            int(binary.shape[0] * 0.9),
            int(binary.shape[0] * 0.75),
            int(binary.shape[0] * 0.6)
        ]

        centers = []
        detected_y = []

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
                detected_y.append(y)

        # 6. Estimation de la position et orientation
        if len(centers) >= 2:

            # Ajustement d'une droite x = a*y + b
            a, b = np.polyfit(detected_y, centers, 1)

            # Prediction de la position de la ligne plus loin
            lookahead_y = int(
                binary.shape[0] * LOOKAHEAD_RATIO
            )

            cx = int(a * lookahead_y + b)

        elif len(centers) == 1:

            # Si une seule ligne est detectee,
            # on utilise son centre sans anticipation
            cx = centers[0]
            lookahead_y = detected_y[0]

        else:
            # Ligne perdue : rabat vers le dernier cote vu, puis arret
            if lost_since is None:
                lost_since = time.time()
            if time.time() - lost_since < MAX_SEARCH_TIME:
                robot.SetMotors(
                    -last_error_sign * SEARCH_SPEED,
                    last_error_sign * SEARCH_SPEED
                )
            else:
                robot.Stop()

            cv2.imshow("Camera", image)
            cv2.imshow("Detection ligne", binary)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # Ligne retrouvee : reinitialisation de la recherche
        lost_since = None

        # 7. Calcul de l'erreur
        error = cx - w / 2

        # Memorisation du dernier cote de la ligne (apres le calcul de l'erreur)
        if abs(error) > 2:
            last_error_sign = 1 if error > 0 else -1

        derivative = error - previous_error

        correction = KP * error + KD * derivative

        previous_error = error

        # 8. Commande des moteurs
        left_speed = BASE_SPEED + STEERING_SIGN * correction
        right_speed = BASE_SPEED - STEERING_SIGN * correction

        left_speed = int(max(
            -MAX_SPEED, min(MAX_SPEED, left_speed)
        ))

        right_speed = int(max(
            -MAX_SPEED, min(MAX_SPEED, right_speed)
        ))

        robot.SetMotors(left_speed, right_speed)

        # 9. Visualisation
        for y, x in zip(detected_y, centers):
            cv2.circle(binary, (x, y), 4, 128, -1)

        cv2.circle(
            binary,
            (max(0, min(w - 1, cx)), lookahead_y),
            7, 200, -1
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
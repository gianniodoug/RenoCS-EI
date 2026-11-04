# huit_PD_rabat_rapide.py : base huit_PD_rabat.py
# + image 320x240, ralentissement en virage, gains adaptes a la vitesse,
#   rabat dans le bon sens, affichage desactivable
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

BASE_SPEED = 100          # vitesse en ligne droite : on peut monter (100, 120...)
MIN_SPEED = 60            # vitesse minimale dans les virages serres
SLOWDOWN = 0.6            # 0 = pas de ralentissement ; 0.6 = -60 % quand la ligne est au bord

# Gains regles a 60 PWM, la ou le suivi marche bien.
# Image 320 px : memes gains que 0.04 / 0.015 a 640 px.
KP = 0.08
KD = 0.03
TUNED_SPEED = 60          # vitesse a laquelle KP / KD ont ete regles

previous_error = 0
MAX_SPEED = 255

SHOW_DEBUG = True         # False pendant les essais rapides : boucle plus rapide

# Paramètres de recherche de ligne
SEARCH_SPEED = 45         # 25 etait trop faible pour faire tourner les moteurs
MAX_SEARCH_TIME = 1.0
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

        # 6. Calcul de l'erreur et correction PD
        if len(centers) > 0:

            # Au moins un barycentre détecté
            cx = int(np.mean(centers))

            error = cx - w / 2
            derivative = error - previous_error

            # Vitesse : on ralentit quand la ligne s'eloigne du centre (virage)
            speed = BASE_SPEED * (1 - SLOWDOWN * min(1.0, abs(error) / (w / 2)))
            speed = max(MIN_SPEED, speed)

            # Correction proportionnelle a la vitesse : meme courbure qu'a 60 PWM
            correction = (KP * error + KD * derivative) * (speed / TUNED_SPEED)
            previous_error = error

            # Memorisation du dernier cote de la ligne
            if abs(error) > 2:
                last_error_sign = 1 if error > 0 else -1

            # Ligne retrouvee : reinitialisation de la recherche
            lost_since = None

            left_speed = speed + correction
            right_speed = speed - correction

            # Limitation des commandes
            left_speed = int(max(
                -MAX_SPEED, min(MAX_SPEED, left_speed)
            ))
            right_speed = int(max(
                -MAX_SPEED, min(MAX_SPEED, right_speed)
            ))

            # 7. Commande des moteurs physiques
            robot.SetMotors(left_speed, right_speed)

            cv2.circle(
                binary,
                (cx, binary.shape[0] // 2),
                5, 128, -1
            )

        else:

            # Aucun barycentre detecte : perte de ligne
            if lost_since is None:
                lost_since = time.time()

            if time.time() - lost_since < MAX_SEARCH_TIME:

                # Rotation sur place vers le dernier cote connu
                # Meme sens que la correction PD (left = speed + correction)
                robot.SetMotors(
                    last_error_sign * SEARCH_SPEED,
                    -last_error_sign * SEARCH_SPEED
                )

            else:
                # Arret si la ligne n'est pas retrouvee
                robot.Stop()

        # 8. Affichage (desactivable : imshow ralentit beaucoup la boucle)
        if not SHOW_DEBUG:
            continue

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
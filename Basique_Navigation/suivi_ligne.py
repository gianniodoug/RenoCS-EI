
import numpy as np
import cv2
import time

from picamera2 import Picamera2
from arduino import arduino

# -------------------------
# Initialisation caméra
# -------------------------

camera = Picamera2()
camera.configure(
    camera.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
)
camera.start()
time.sleep(2)

# -------------------------
# Initialisation Arduino
# -------------------------

robot = arduino('/dev/ttyACM0')

if not robot.connect():
    print("Erreur de connexion Arduino")
    camera.stop()
    raise SystemExit

# -------------------------
# Paramètres du suivi
# -------------------------

BASE_SPEED = 100
KP = 0.04
KD = 0.015
previous_error = 0

MAX_SPEED = 200

# -------------------------
# Commande des moteurs
# -------------------------

def set_motors(left_speed, right_speed):

    left_speed = int(max(-MAX_SPEED, min(MAX_SPEED, left_speed)))
    right_speed = int(max(-MAX_SPEED, min(MAX_SPEED, right_speed)))

    # Ordre : avant droit, avant gauche,
    # arrière droit, arrière gauche
    robot.SetMotors(
        right_speed,
        left_speed,
        right_speed,
        left_speed
    )

# -------------------------
# Boucle principale
# -------------------------

try:
    while True:

        # 1. Récupération de l'image
        image = camera.capture_array()

        # 2. Conversion en niveaux de gris
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        h, w = gray.shape

        # 3. Sélection de la zone basse
        roi = gray[int(h * 0.70):h, :]

        # 4. Détection de la piste blanche
        _, binary = cv2.threshold(
            roi, 80, 255, cv2.THRESH_BINARY
        )

        # 5. Calcul du barycentre
        moments = cv2.moments(binary)

        if moments["m00"] > 0:

            cx = int(moments["m10"] / moments["m00"])

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
            # Ligne perdue : arrêt de sécurité
            robot.Stop()

        # Affichage pour le débogage
        cv2.imshow("Camera", image)
        cv2.imshow("Detection ligne", binary)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    # Arrêt et libération des ressources
    robot.Stop()
    robot.disconnect()
    camera.stop()
    cv2.destroyAllWindows()

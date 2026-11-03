import numpy as np
import cv2
from controller import Robot
'''
-------------------------
Initialisation Webots
-------------------------
'''
TIME_STEP = 64
robot = Robot()
camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)
'''
Ordre des moteurs du code initial :
Motor2 = avant gauche
Motor1 = avant droit
Motor4 = arrière gauche
Motor3 = arrière droit
'''
motors = []
motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
for name in motors_names:
    motor = robot.getDevice(name)
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)
    motors.append(motor)
'''
-------------------------
Paramètres du suivi
-------------------------
'''
BASE_SPEED = 4.0
KP = 0.04
KD = 0.015
previous_error = 0
MAX_SPEED = 10.0
'''
-------------------------
Commande des moteurs
-------------------------
'''
def set_motors(left_speed, right_speed):
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
    motors[0].setVelocity(left_speed)
    motors[2].setVelocity(left_speed)
    motors[1].setVelocity(right_speed)
    motors[3].setVelocity(right_speed)
'''
-------------------------
Boucle principale
-------------------------
'''
while robot.step(TIME_STEP) != -1:
    # 1. Récupération de l'image
    cameraData = camera.getImage()
    image = np.frombuffer(
    cameraData, np.uint8
    ).reshape(
    (camera.getHeight(), camera.getWidth(), 4)
    )
    # 2. Conversion en niveaux de gris
    gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    h, w = gray.shape
    # 3. Sélection de la zone basse de l'image
    roi = gray[int(h * 0.70):h, :]
    # 4. Détection de la piste blanche
    _, binary = cv2.threshold(
    roi, 80, 255, cv2.THRESH_BINARY
    )
    # 5. Calcul du barycentre de la piste
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
        set_motors(0, 0)
    # Affichage
    cv2.imshow("Camera", image)
    cv2.imshow("Detection ligne", binary)
    cv2.waitKey(1)
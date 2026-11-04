import time
from arduino import arduino
from utilitaries import sendCmdi, retrCmdi

PORT = "/dev/ttyACM0"

robot = arduino(PORT)

try:
    if not robot.connect():
        raise RuntimeError("Impossible de se connecter à l'Arduino")

    print("Connexion Arduino réussie.")
    print("Test du capteur ultrason.")
    print("Place un obstacle devant le capteur.")
    print("Appuie sur Ctrl+C pour arrêter.\n")

    while True:
        # Demande de mesure ultrason
        sendCmdi(robot.serialport, b'S', 0, 0, 0, 0)

        # Réception des 4 entiers
        result = retrCmdi(robot.serialport)

        distance = result[0]

        if 4 <= distance <= 500:
            print(f"Distance : {distance} cm")
        else:
            print(f"Mesure invalide : {distance}")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nArrêt du test.")

except Exception as e:
    print("Erreur :", e)

finally:
    robot.disconnect()
    print("Arduino déconnecté.")
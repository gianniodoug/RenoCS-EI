# General imports

import numpy as np
import cv2

# Specific imports for Webots

from controller import Robot
from controller import Camera

#####################
# Robot setup and control
#####################

speed_constant = 10  # Speed constant to control the speed of the robot




current_state = "BACK"




def set_motors_velocities(state, speed_constant):
    if state == "FRONT":
        motors[0].setVelocity(1 * speed_constant)
        motors[1].setVelocity(1 * speed_constant)
        motors[2].setVelocity(1 * speed_constant)
        motors[3].setVelocity(1 * speed_constant)
    elif state == "BACK":
        motors[0].setVelocity(-1 * speed_constant)
        motors[1].setVelocity(-1 * speed_constant)
        motors[2].setVelocity(-1 * speed_constant)
        motors[3].setVelocity(-1 * speed_constant)
    else:  
        motors[0].setVelocity(0)
        motors[1].setVelocity(0)
        motors[2].setVelocity(0)
        motors[3].setVelocity(0)


#####################
# Webots setup
#####################

TIME_STEP = 64
robot = Robot()

# Enable camera
camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)

# Listing the wheels
motors = []
motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
for i in range(len(motors_names)):
    motors.append(robot.getDevice(motors_names[i]))
    motors[i].setPosition(float("inf"))
    motors[i].setVelocity(0.0)

time0 = robot.getTime()

#####################
# Main loop
#####################

while robot.step(TIME_STEP) != -1:
    #####################
    # Image processing
    #####################

    # Read image from webots camera

    cameraData = camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )

    # Read/Write issue
    image = np.copy(image)
    h, w = image.shape[:2]

    #Traitement image
    #
    #
    #
    #
    #
    #
    #
    #
    #
    im2 = image

    # Prise de décision
    if robot.getTime()-time0 >2:
        time0 = robot.getTime()
        if current_state == "BACK":
            current_state = "FRONT"
        else:
            current_state = "BACK"
    set_motors_velocities(current_state, speed_constant)



    cv2.imshow("Image traitee", im2)
    cv2.waitKey(1)

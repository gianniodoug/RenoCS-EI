#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 32             // ms
#define SCAN_SPEED 0.25          // radians per second
#define MAX_ANGLE M_PI/2         // 90°
#define MIN_ANGLE -M_PI/2        // 90°
#define TOLERANCE 0.01           // radians (précision d'arrêt)

int main() {
  wb_robot_init();

  // Récupération du moteur
  WbDeviceTag servo = wb_robot_get_device("servomoteur");

  // Récupération du capteur de position
  WbDeviceTag capteur = wb_robot_get_device("Capteur_servo");
  wb_position_sensor_enable(capteur, TIME_STEP);

  // Initialisation : on commence vers MAX_ANGLE
  double current_target = MAX_ANGLE;
  wb_motor_set_velocity(servo, SCAN_SPEED);
  wb_motor_set_position(servo, current_target);

  while (wb_robot_step(TIME_STEP) != -1) {
    double position = wb_position_sensor_get_value(capteur);

    // Quand la position est atteinte, on inverse la direction
    if (fabs(position - current_target) < TOLERANCE) {
      if (current_target == MAX_ANGLE)
        current_target = MIN_ANGLE;
      else
        current_target = MAX_ANGLE;

      wb_motor_set_position(servo, current_target);
    }
  }

  wb_robot_cleanup();
  return 0;
}


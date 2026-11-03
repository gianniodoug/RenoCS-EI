#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 100

/* full Bluno wiring */
const int ServofrontPin = 26;
const int IR_pin = A6;

const int dirMotor1 = 4;  // motor direction pin
const int vitMotor1 = 3;  // motor PWM pin
const int Encoder1B = 18;  // encoder speed pin -> white
const int Encoder1A = 31;  // encoder direction pin -> yellow

const int dirMotor2 = 12; // motor direction pin
const int vitMotor2 = 11; // motor PWM pin
const int Encoder2B = 19;  // encoder speed pin -> white
const int Encoder2A = 32;  // encoder direction pin -> yellow

const int dirMotor3 = 8;  // motor direction pin
const int vitMotor3 = 5;  // motor PWM pin
const int Encoder3B = 20;  // encoder speed pin -> white
const int Encoder3A = 33;  // encoder direction pin -> yellow

const int dirMotor4 = 7;  // motor direction pin
const int vitMotor4 = 6;  // motor PWM pin
const int Encoder4B = 21;  // encoder speed pin -> white
const int Encoder4A = 34;  // encoder direction pin -> yellow

#endif

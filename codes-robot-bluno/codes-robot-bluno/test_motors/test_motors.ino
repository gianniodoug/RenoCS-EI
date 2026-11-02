// #
// # Editor(s)  :  Morgan ROGER from Centralesupelec
// # Last       :  Morgan ROGER from Centralesupelec
// # Version    :  01.11.2026

// # Description:
// # The sketch for testing the motors and encoders on the ST5 VAC robot

// # Connections:
// #        See pin numbers in file Parameters.h


#include "Motor.h"
#include "Parameters.h"

int value[4] = {0,0,0,0};  // 'signed PWM' values between -255 and 255
int setpoint[4] = {0,0,0,0}; // PWM setpoints between -255 and 255
unsigned char deltaMax = 10;  // maximum gap between 2 successive values

#define numtask 2    // number of tasks
int deltask[numtask] = {0, 500};  // array defining the period of all tasks
int timtask[numtask] ;         // array with the next execution time of all tasks
bool taskon[numtask] = {false, false}; // array with activation state of all tasks
void task0();  void task1();                // declaring the function for each task
void (*NameTask[numtask])() = {  task0, task1  }; // array with the name of all task functions

Motor FRmotor(vitMotor1,dirMotor1,0); // front right
Motor FLmotor(vitMotor2,dirMotor2,0); // front left
Motor RRmotor(vitMotor3,dirMotor3,0); // rear right
Motor RLmotor(vitMotor4,dirMotor4,0); // rear left
long int volatile CoderCount[4] = {0,0,0,0};

char CharIn;
char c;

void TaskOn(int task) {
  taskon[task]=true;  
  timtask[task] = (int)millis()+deltask[task]; 
}

void setup() {
  Serial.begin(115200);  // init the Serial port and set the baud rate
  Serial.setTimeout(2);  // setting a short timeout of 2 ms for the parseInt()
  FRmotor.setvalue(0);
  FLmotor.setvalue(0);
  RRmotor.setvalue(0);
  RLmotor.setvalue(0);
  TaskOn(0);
  TaskOn(1);
  attachInterrupt(digitalPinToInterrupt(Encoder1B), FRcoderISR, FALLING); // interrupt attachment for encoder routines
  attachInterrupt(digitalPinToInterrupt(Encoder2B), FLcoderISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(Encoder3B), RRcoderISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(Encoder4B), RLcoderISR, FALLING); 
}

void loop() {
  for (int k=0;k<numtask;k++)
    if (taskon[k]) 
      if ((((int)millis()-timtask[k])>0) || (deltask[k]==0))  // if execution time has come
      {
        NameTask[k]() ;                   // launching task #k
        timtask[k]=timtask[k]+deltask[k] ;
      } 
}

void cmd_M() {
  c = Serial.peek() ;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint[0] = Serial.parseInt(); // reading motor setpoint value
  else
    setpoint[0] = 0 ; // default: stop motor
  Serial.print("Front right motor setpoint: ");
  Serial.println(setpoint[0]);
}

void cmd_N() {
  c = Serial.peek() ;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint[1] = Serial.parseInt(); // reading motor setpoint value
  else
    setpoint[1] = 0 ; // default: stop motor
  Serial.print("Front left motor setpoint: ");
  Serial.println(setpoint[1]);
}

void cmd_O() {
  c = Serial.peek() ;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint[2] = Serial.parseInt(); // reading motor setpoint value
  else
    setpoint[2] = 0 ; // default: stop motor
  Serial.print("Rear right motor setpoint: ");
  Serial.println(setpoint[2]);
}

void cmd_P() {
  c = Serial.peek() ;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint[3] = Serial.parseInt(); // reading motor setpoint value
  else
    setpoint[3] = 0 ; // default: stop motor
  Serial.print("Rear left motor setpoint: ");
  Serial.println(setpoint[3]);
}

void cmd_I() {
  Serial.print("Front right encoder value: ");
  Serial.println(CoderCount[0]);
}

void cmd_J() {
  Serial.print("Front left encoder value: ");
  Serial.println(CoderCount[1]);
}

void cmd_K() {
  Serial.print("Rear right encoder value: ");
  Serial.println(CoderCount[2]);
}

void cmd_L() {
  Serial.print("Rear left encoder value: ");
  Serial.println(CoderCount[3]);
}

void task0() {
  if (Serial.available() > 0) {
    CharIn = Serial.read();
    delay(2); // essential but not too long
    switch(CharIn) {
      case 'M':  cmd_M();  break;    // orders for motor PWM setpoints
      case 'N':  cmd_N();  break;
      case 'O':  cmd_O();  break;
      case 'P':  cmd_P();  break;
      case 'I':  cmd_I();  break;    // requests for encoder values
      case 'J':  cmd_J();  break;
      case 'K':  cmd_K();  break;
      case 'L':  cmd_L();  break;
    }
  }
}

void task1() {
  for (int i=0; i<4; i++) {
    if (value[i] < setpoint[i])
      if (value[i] > (setpoint[i]-deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] += deltaMax;
    else
      if (value[i] < (setpoint[i]+deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] -= deltaMax;
  }
  
  FRmotor.setvalue(value[0]); // after updating the value to improve reactivity to new setpoints 
  FLmotor.setvalue(value[1]);
  RRmotor.setvalue(value[2]);
  RLmotor.setvalue(value[3]);
}

void FRcoderISR() {
  CoderCount[0] += ((digitalRead(Encoder1A) == HIGH) ? +1 : -1);       
}

void FLcoderISR() {
  CoderCount[1] += ((digitalRead(Encoder2A) == LOW) ? +1 : -1);       
}

void RRcoderISR() {
  CoderCount[2] += ((digitalRead(Encoder3A) == HIGH) ? +1 : -1);       
}

void RLcoderISR() {
  CoderCount[3] += ((digitalRead(Encoder4A) == LOW) ? +1 : -1);       
}


// #
// # Editor(s)  :  Morgan ROGER from Centralesupelec
// # Last       :  Morgan ROGER from Centralesupelec
// # Version    :  25.10.2026

// # Description:
// # The sketch for testing the servomotor on the ST5 VAC robot

// # Connections:
// #        Front servo  -> Digital pin 26 (triple connector orange, red, brown)


#include "MyServo.h"

/* used Arduino pins */
const int  frontPin = 26;

/* servo management */
MyServo frontServo(frontPin, 20, 160, 0);    // create servo objects

/* task management */
#define numtask 2    // number of tasks
int deltask[numtask] = {0, 500};  // array defining the period of all tasks
int timtask[numtask] ;         // array with the next execution time of all tasks
bool taskon[numtask] = {false, false}; // array with activation state of all tasks
void task0();  void task1();                // declaring the function for each task
void (*NameTask[numtask])() = {  task0, task1  }; // array with the name of all task functions

void TaskOn(int task) {
  taskon[task]=true;  
  timtask[task] = (int)millis()+deltask[task]; 
}


/* SETUP */
void setup() {
  Serial.begin(115200);  // init the Serial port and set the baud rate
  frontServo.start();
  frontServo.setpos(90);
  TaskOn(0);              // activate task #0
  TaskOn(1);              // activate task #1
}

/* LOOP */
void loop() {
  for (int k=0;k<numtask;k++)
    if (taskon[k]) 
      if ((((int)millis()-timtask[k])>0) || (deltask[k]==0))  // if execution time has come
      {
        NameTask[k]() ;                   // launching task #k
        timtask[k]=timtask[k]+deltask[k] ;
      }
}

/* TASK #0 = read command from serial */
void task0() {
  char CharIn;
  if (Serial.available() > 0) {
    CharIn = Serial.read();
    delay(2); // essential but not too long
    switch(CharIn) {
      case 'P':  cmd_P();  break;    // setting front servo position
      case 'S':  cmd_S();  break;    // setting front servo sweep step
    }
  }
}

/* TASK #1 = sweep the servo */
void task1() {
  frontServo.advance();
}


/* Command P = set servo position */
void cmd_P() {
  char c = Serial.peek() ;
  int setpoint;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint = Serial.parseInt(); // getting the numerical value
  else
    setpoint = 90 ; // default: center position
  frontServo.setpos(setpoint);
  Serial.print("Front servo position set to ");
  Serial.println(setpoint);
}

/* Command S = set servo sweep step */
void cmd_S() {
  char c = Serial.peek() ;
  int setpoint;
  if ((c=='-') || ((c>='0') && (c<='9'))) 
    setpoint = Serial.parseInt(); // getting the numerical value
  else
    setpoint = 0 ; //default: stop sweep
  frontServo.setspeed(setpoint);
  Serial.print("Front servo speed set to ");
  Serial.println(setpoint);
}


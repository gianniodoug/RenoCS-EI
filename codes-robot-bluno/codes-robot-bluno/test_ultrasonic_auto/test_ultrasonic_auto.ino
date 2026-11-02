// #
// # Editor(s)  :  Erwan LIBESSART, Morgan ROGER from Centralesupelec
// # Last       :  Morgan ROGER from Centralesupelec
// # Version    :  31.10.2026

// # Description:
// # The sketch for testing the ultrasonic sensor URM37 in auto mode on the ST5 VAC robot

// # Connections:
// #        URM echo output    -> Digital pin 2 (green), 5V (red), GND (black)
// #        front servo        -> Digital pin 26 (triple connector orange, red, brown)


#include "MyServo.h"

/* used Arduino pins */
const int URM_ECHO = 2; // URM echo output : PWM 0－25000µs，every 50µs represent 1cm
//const int URM_TRIG = 10; // URM trigger pin (unused in auto mode)
const int  servoPin = 26;  // pin number of the servo

/* sensor management */
unsigned long distanceInCm = 0;

/* servo management */
MyServo frontServo(servoPin, 20, 160, 0);    // create servo objects

/* task management */
#define numtask 3    // number of tasks
int deltask[numtask] = {0, 500, 500};  // array defining the period of all tasks
int timtask[numtask] ;         // array with the next execution time of all tasks
bool taskon[numtask] = {false, false, false}; // array with activation state of all tasks
void task0();  void task1();  void task2();        // declaring the function for each task
void (*NameTask[numtask])() = {  task0, task1, task2  }; // array with the name of all task functions

void TaskOn(int task) {
  taskon[task]=true;  
  timtask[task] = (int)millis()+deltask[task]; 
}


/* SETUP */
void setup() {
  Serial.begin(115200);  // init the Serial port and set the baud rate
  URMsetup();
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

/* TASK #1 = distance measurement with URM */
void task1() {
  // useless in auto mode: a low pull on pin COMP/TRIG  triggering a sensor reading
  //digitalWrite(URM_TRIG, LOW);
  //digitalWrite(URM_TRIG, HIGH);
  unsigned long pulseLength = pulseIn(URM_ECHO,LOW);  // ECHO pin will output low pulses
  distanceInCm = pulseLength/50;  // every 50µs low level stands for 1cm
  Serial.println();
  if(distanceInCm>900) {             // the reading is invalid.
    Serial.println("Invalid");
  } else {
    Serial.println(distanceInCm);
  }
}

/* TASK #2 = sweep the servo */
void task2() {
  frontServo.advance();
}


/* setup of the URM */
void URMsetup() {
  //pinMode(URM_TRIG,OUTPUT);          // useless in auto mode
  //digitalWrite(URM_TRIG,HIGH);       // Set to HIGH
  pinMode(URM_ECHO, INPUT);
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
  if ((c=='-') || ((c>='0') && (c<='9'))) {
    setpoint = Serial.parseInt(); // getting the numerical value
    TaskOn(2);
    frontServo.setspeed(setpoint);
    Serial.print("Front servo speed set to ");
    Serial.println(setpoint);
  }
  else { //default: switch sweep state on/off
    if (taskon[2]) {
      taskon[2] = false; 
      Serial.println("Sweep off");
    }
    else {
      TaskOn(2);
      Serial.println("Sweep on");
    }
  }
}


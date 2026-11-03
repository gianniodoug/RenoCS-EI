#include "Arduino.h" 
#include "parameters.h"
#include "Motor.h"
#include "MyServo.h"
#include "Commandes.h"

// on définit ici toutes les variables auxquelles doivent accéder les taches
void init_arduino();
extern Motor FRmotor;
extern Motor FLmotor;
extern Motor RRmotor;
extern Motor RLmotor;
extern long int volatile CoderCount[4] ; 
extern bool obst;
extern MyServo frontservo;

int value[4] = {0,0,0,0};  // 'signed PWM' values between -255 and 255
int setpoint[4] = {0,0,0,0}; // PWM setpoints between -255 and 255
unsigned char deltaMax = 10;  // maximum gap between 2 successive values

// gestion du multitache. définition de la période de chaque tache
// definition du nombre de taches maximales
#define numtask 3
unsigned long deltask[numtask]={0,20,20} ;
unsigned long timtask[numtask] ;
unsigned long lastMotorCommandMs=0;
const unsigned long MOTOR_WATCHDOG_MS=350UL;
void refreshMotorWatchdog() { lastMotorCommandMs=millis(); }
bool taskon[numtask]={true,true,true} ;
void task0(); // tache permanente de lecture de la liaison série 
void task1(); // tache du chien de garde
void task2(); // tache de transition douce pour les moteurs


void (*NameTask[numtask])() = {  task0,task1,task2  };

///////////////////////////////////////////////////////////////////////////
//   mise en route et arret des taches périodiques
/////////////////////////////////////////////////////////////////////////
void TaskOn(int task)
{ taskon[task]=true;  
  timtask[task] = millis()+deltask[task]; 
}

void TaskOff(int task)
{ taskon[task]=false;  
}

void TaskOffAll() {
  for (int k=1; k<numtask; k++)
    taskon[k]=false ;
}

bool TaskState(int task) {
  return(taskon[task]);
}

  // lancement des differentes taches périodiques
void launch_tasks() {
  for (int k=0;k<numtask;k++)
    if (taskon[k]) 
      if (((long)(millis()-timtask[k])>=0) || (deltask[k]==0))   // si on a dépassé le temps du prochain lancement ou si tache permanente
      {
        NameTask[k]() ;                   // on lance la tache k
        timtask[k]=timtask[k]+deltask[k] ;
      }
}

////////////////////////////////////////////////////////////////////
//
// LES TACHES PERIODIQUES
//
////////////////////////////////////////////////////////////////////

extern bool ConnOn ;

// tache permanente de lecture de la liaison série  
void task0() {
  char CharIn;
  if (Serial.available() > 0) {
    // on lit le premier caractère qui arrive
    CharIn = Serial.read();
    if ((CharIn=='A') || (ConnOn)) decod_serial(CharIn);  // on ne fait un decodage de commande que si on est connecté
  }

}

// tache du chien de garde  
void task1() {
  if ((millis()-lastMotorCommandMs) > MOTOR_WATCHDOG_MS) {
    for (int i=0;i<4;i++) {setpoint[i]=0; value[i]=0;}
    FRmotor.setvalue(0); FLmotor.setvalue(0);
    RRmotor.setvalue(0); RLmotor.setvalue(0);
  }
}

void task2() {
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


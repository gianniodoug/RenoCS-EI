//////////////////////////////////////////////////////-
// Ce programme a été développé à CENTRALESUPELEC
// Merci de conserver ce cartouche
// Copyright  (c) 2026  CENTRALE-SUPELEC   
// Département Electronique et électromagnétisme
// //////////////////////////////////////////////////-
//
// fichier : Arduino_commande_moteurs
// auteur  : P.BENABES   
// éditeur : M.ROGER
// Copyright (c) 2026 CENTRALE-SUPELEC
// Revision: 2.1  Date: 01/11/2026
//
// //////////////////////////////////////////////////-
//
// SKETCH DESCRIPTION:
// program running on the Bluno board
// for the ST5 VAC mobile robot
//
//////////////////////////////////////////////////////

#include "parameters.h"   // paramètres de la carte (N° de broches, etc ...)
#include "Motor.h"        // pour utiliser les onjets Moteur
#include "MyServo.h"      // pour utiliser les objets servo moteur
//#include "Screen.h"       // pour utiliser les objets ecran
#include "Sercomm.h"      // toutes les fonctions de la liaison série
#include "Task.h"         // pour la gestion des taches
#include "Commandes.h"    // toutes les commandes reçues par la liaison série

///////////////////////////////////////////////////////////////////////////
//
//    DEFINITION DES VARIABLES GLOBALES ET OBJETS
//
/////////////////////////////////////////////////////////////////////////

bool obst=false ;    // obstacle détecté
bool ConnOn ;        // indique si la carte est connectée
char feedback ;      // si non nul indique que les commandes doivent renvoyer un acquitement
char commode=0 ;     // indique le mode de communication 0=tout en ASCII, 1 les commandes sont en binaire et les retours en ascii, 2 tout est en binaire


// variables de la gestion des moteurs 
Motor FRmotor(vitMotor1,dirMotor1,0); // front right
Motor FLmotor(vitMotor2,dirMotor2,0); // front left
Motor RRmotor(vitMotor3,dirMotor3,0); // rear right
Motor RLmotor(vitMotor4,dirMotor4,0); // rear left

// les compteurs incrémentaux
long int volatile CoderCount[4] = {0,0,0,0};

// gestion du servomoteur
MyServo frontservo(ServofrontPin,30,150,10);


///////////////////////////////////////////////////////////////////////////
//
//   INITIALISATION DE L'ARDUINO
//
/////////////////////////////////////////////////////////////////////////

void init_arduino() {

  FRmotor.setvalue(0);
  FLmotor.setvalue(0);
  RRmotor.setvalue(0);
  RLmotor.setvalue(0);
  frontservo.start() ;
  frontservo.setpos((frontservo.getmin()+frontservo.getmax())/2);

  TaskOffAll();
  obst=false ;
  TaskOn(0) ;     // on allume la tache permanente de lecture de la liaison série
  TaskOn(2) ;     // on allume la tache transition douce pour les moteurs
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT); // timeout pour attendre une fin de message

  init_arduino() ;

  attachInterrupt(digitalPinToInterrupt(Encoder1B), FRcoderISR, FALLING); // interrupt attachment for encoder routines
  attachInterrupt(digitalPinToInterrupt(Encoder2B), FLcoderISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(Encoder3B), RRcoderISR, FALLING); 
  attachInterrupt(digitalPinToInterrupt(Encoder4B), RLcoderISR, FALLING); 
}

////////////////////////////////////////////////////////////
//
// la boucle principale
//
/////////////////////////////////////////////////////////////


void loop() {
  launch_tasks() ;
}

////////////////////////////////////////////////////////////
//
// les routines d'interruption
//
////////////////////////////////////////////////////////////

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


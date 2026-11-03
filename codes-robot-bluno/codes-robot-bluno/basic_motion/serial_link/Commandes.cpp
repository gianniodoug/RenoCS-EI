#include "Arduino.h" 
#include "parameters.h"
#include "Sercomm.h"
#include "Motor.h"
#include "MyServo.h"
//#include "Screen.h"
#include "Task.h"

// on définit ici toutes les variables auxquelles doivent accéder les taches
extern Motor FRmotor;
extern Motor FLmotor;
extern Motor RRmotor;
extern Motor RLmotor;

extern int value[4];  // 'signed PWM' values between -255 and 255
extern int setpoint[4]; // PWM setpoints between -255 and 255

extern long int volatile CoderCount[] ; 
extern bool obst;
extern MyServo frontservo;
extern char feedback ;
extern char commode ;
extern bool ConnOn ;

extern void init_arduino() ;

//////////////////////////////////////////////////////////////////////////////
//
// LES CODES DE TRAITEMENT DES COMMANDES
//
///////////////////////////////////////////////////////////////////////////////

void dummy() {  
  Serial.println(F("ER")); }


////////////////////////////////////////////
//  Commande A : connection de la carte  :
////////////////////////////////////////////

void CONNECT_code() {
  delay(1); // indispensable et pas trop long sinon le caractère suivant n'est pas arrivé
  char c=GetChar(0);
  feedback=c-'0' ;
  commode= GetChar(0);
  if (commode>0) commode-='0'; else commode=0 ;
  if (ConnOn==false)     // on n'est pas encore connecté
    { 
    ConnOn = true ;
    init_arduino() ;
    }
  RetAcquit(F("OK Arduino connecte version 1.0 en mode "),commode);
}

////////////////////////////////////////////
// Commande a :  deconnection de la carte
////////////////////////////////////////////

void DISCONNECT_code() {
  init_arduino();
  Serial.println(F("OK Arduino deconnecte"));
  ConnOn = false ;
}

////////////////////////////////////////////
// Commande B :  commande vide
////////////////////////////////////////////
void empty() {  Serial.println(F("OK")); }

////////////////////////////////////////////
// Commande c :  commande d'1 seul moteur
////////////////////////////////////////////

void SINGLEMOTOR_code() {
  char s ;
  //Motor *mx=0 ;
  int nivM ;

  delay(1); // indispensable et pas trop long
  char m=GetChar(0);
  //if (m=='1')   mx=&FRmotor; 
  //else if (m=='2')  mx=&FLmotor; 
  //else if (m=='3')  mx=&RRmotor; 
  //else if (m=='4')  mx=&RLmotor; 
  //if (mx)
  { nivM=GetInt(0) ;              // recupere la valeur de la commande
    GetInt(0) ;
    GetLong(0);
    //mx->setvalue(nivM) ;
    setpoint[m-'0'-1] = nivM;

    if (obst==false)
        RetAcquit(F("OK Moteur mis à la consigne : "),(int)(m-'0'),nivM);
    else
        RetAcquit(F("OB stacle détecté moteur non allumé"));
    }
  //else
  //  { Serial.readString() ;
  //  Serial.println(F("Erreur commande incomplète")); 
  //}
}


////////////////////////////////////////////
// Commande C :  commande des 4 moteurs
////////////////////////////////////////////
void QUADMOTOR_code() {
  int nivM1,nivM2,nivM3,nivM4 ;
  delay(1); // indispensable et pas trop long

  // on recupere les parametres
  nivM1=GetInt(0) ;   // on lit la valeur du premier moteur
  nivM2=GetInt(nivM1) ; // on lit la valeur du deuxième moteur
  nivM3=GetInt(nivM2) ; // on lit la valeur du troisième moteur
  nivM4=GetInt(nivM3) ; // on lit la valeur du quatrième moteur

  // on envoie la commande aux 4 moteurs
  setpoint[0] = nivM1;
  setpoint[1] = nivM2;
  setpoint[2] = nivM3;
  setpoint[3] = nivM4;

  
  // Arresto immediato quando tutte le consigne sono zero
  if (nivM1 == 0 && nivM2 == 0 &&
      nivM3 == 0 && nivM4 == 0) {

    for (int i = 0; i < 4; i++) {
      value[i] = 0;
      setpoint[i] = 0;
    }

    FRmotor.setvalue(0);
    FLmotor.setvalue(0);
    RRmotor.setvalue(0);
    RLmotor.setvalue(0);
  }

  
  //reponse de la commande
  if (obst==false)
        RetAcquit(F("OK Moteurs mis aux consignes : "),nivM1,nivM2,nivM3,nivM4);
  else
        RetAcquit(F("OB stacle détecté moteur non allumé"));

}


//////////////////////////////////////////////////////////
// Commande G : pour envoyer le servomoteur à une certaine position
//////////////////////////////////////////////////////////

void SERVO_code() {
  delay(1); // indispensable et pas trop long
  int pos;

  // onn recupere les paramètres 
  pos=GetInt(0) ;  // on recupere la vitesse pour positionner le moteur
  GetInt(0); GetLong(0);
  frontservo.setpos(pos);
   
  // on renvoie la réponse
  RetAcquit(F("OK servomoteur en "),pos);    
}

//////////////////////////////////////////////////////////
// Commande g : pour régler la consigne min et max des servomoteurs
//////////////////////////////////////////////////////////
void SERVO_minmax() {
  int min,max ;
  delay(1); // indispensable et pas trop long
  
  // onn recupere les paramètres 
  min=GetInt(0) ;  // on recupere la vitesse pour positionner le moteur
  max=GetInt(0);
  GetLong(0);
  frontservo.setmin(min);
  frontservo.setmax(max);
  
  // on renvoie la réponse
  RetAcquit(F("servomoteur min max = "),frontservo.getmin(), frontservo.getmax());
}

//////////////////////////////////////////////////////////
// Commande I,i : mise en route de la protection moteur
// chaque appel à cette commande réinitialise la détection et autorise le redémarrage des moteurs 
//////////////////////////////////////////////////////////

void PROTECT_IR_code() {
  //delay(1); // indispensable et pas trop long
  //char m=GetChar(0);
  //if (m=='0')   { TaskOff(2) ; obst=false ;   }
  //else if (m=='1')  { TaskOn(2) ;  obst=false ; }    
  //if (feedback==1) Serial.println("OK");
  //if (feedback==2)
  //{  Serial.print(F("OK protection moteur : "));
  //    Serial.println(TaskState(2));    // mise en route de la tache de protection
  //}
}


//////////////////////////////////////////////////////////////////////////////
//
// LES CODES DE TRAITEMENT DES QUESTIONS
//
///////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// question N : renvoie la position des 2 encodeurs avant
//////////////////////////////////////////////////////////
void  ENCODERS_FRONT_code() {
   Ret2l(CoderCount[0],CoderCount[1]);
}

//////////////////////////////////////////////////////////
// question n : renvoie la position des 2 encodeurs arriere
//////////////////////////////////////////////////////////
void  ENCODERS_REAR_code() {
   Ret2l(CoderCount[2],CoderCount[3]);
}

/////////////////////////////////////////////////////////////////////
// question O,o : renvoie le temps courant et la position d'un encodeur
/////////////////////////////////////////////////////////////////////

void  ENCODER_TIME_code() {
  long v2 ;
  delay(1); // indispensable et pas trop long sinon le caractère suivant n'est pas arrivé
  char c=GetChar(0);
  if (c=='1') 
    v2=CoderCount[0]; 
  else if (c=='2')
    v2=CoderCount[1];
  else if (c=='3')
    v2=CoderCount[2];
  else if (c=='4')
    v2=CoderCount[3];
  Ret2l(millis(),v2);
}

/////////////////////////////////////////////////////////////////////
// question P,p : renvoie la vitesse des 4 moteurs
/////////////////////////////////////////////////////////////////////

void SPEED_QUAD_code() {
  Ret4i(FRmotor.getspeed(),FLmotor.getspeed(),RRmotor.getspeed(),RLmotor.getspeed());
}


/////////////////////////////////////////////////////////////////////
// question R,r : renvoie le temps courant et la valeur du capteur infrarouge
/////////////////////////////////////////////////////////////////////

void  INFRARED_TIME_code() {
  Ret2li(millis(),analogRead(IR_pin));
}

/////////////////////////////////////////////////////////////////////
// question S,s : renvoie la valeur du capteur ultrasons
/////////////////////////////////////////////////////////////////////
void  ULTRASON_code() {
 // A COMPLETER
}

/////////////////////////////////////////////////////////////////////
// question T,t : renvoie la commande sur les moteurs
/////////////////////////////////////////////////////////////////////
void  VALMOTOR_code() {
  Ret4i(FRmotor.getvalue(),FLmotor.getvalue(),RRmotor.getvalue(),RLmotor.getvalue());
}

/////////////////////////////////////////////////////
// LE BIG TABLEAU D'AIGUILLAGE DES FONCTIONS
/////////////////////////////////////////////////////


void (*UpperFn[20])() = {           // tableau des fonctions pour un code en majuscule
  CONNECT_code,             // A
  empty,                    // B
  QUADMOTOR_code,           // C
  dummy,                    // D
  dummy,                    // E
  dummy,                    // F
  SERVO_code,               // G
  dummy,                    // H
  PROTECT_IR_code,dummy,dummy,dummy, // I,J,K,L
  dummy,                    // M
  ENCODERS_FRONT_code,        // N
  ENCODER_TIME_code,       // O  
  SPEED_QUAD_code,dummy,    // P,Q
  INFRARED_TIME_code,       // R
  ULTRASON_code,            // S
  VALMOTOR_code             // T
  };

void (*LowerFn[20])() = {             // tableau des fonctions pour un code en minuscule
  DISCONNECT_code,           // a
  dummy,                     // b
  SINGLEMOTOR_code,          // c
  dummy,                    // d
  dummy,                    // e
  dummy,                    // f
  SERVO_minmax,             // g
  dummy,                    // h
  PROTECT_IR_code,dummy,dummy,dummy, // i,J,K,L
  dummy,                     // m
  ENCODERS_REAR_code,         // n
  ENCODER_TIME_code,         // o     // renvoie la position des encodeurs et arrete le moteur
  SPEED_QUAD_code,dummy,     // p,q
  INFRARED_TIME_code,        // r
  ULTRASON_code,             // s
  VALMOTOR_code              // t
  };


// fonction de décodage des messages et exécution des codes correspondants 
void decod_serial(char CharIn)
{
    if ((CharIn>='A') && (CharIn<='T'))       // pour les fonctions 'majuscule'
      UpperFn[CharIn-'A']() ;
    else if ((CharIn>='a') && (CharIn<='t'))  // pour les fonctions 'miniscule'
      LowerFn[CharIn-'a']() ;
    else
      dummy();
}


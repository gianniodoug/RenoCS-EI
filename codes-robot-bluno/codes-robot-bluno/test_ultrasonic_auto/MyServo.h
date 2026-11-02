#ifndef MyServo_h
#define MyServo_h
#include "Arduino.h" 
#include <Servo.h>

class MyServo {
  public:
    MyServo(int pin, int min, int max, int speed); // constructeur du servomoteur
    void start();             // mise en connexion du servomoteur
    void setpos(int pos);     // definit la position du servo moteur
    void setmin(int min);     // definit la position minimale
    void setmax(int max);     // definit la position maximale
    void setspeed(int speed); // definit la vitesse de mouvement
    int  getmin();            // recupere la position minimale
    int  getmax();            // recupere la position maximale
    void advance();           // fait avancer le servo d'un pas
  private:
    Servo _MonServo ; 
    int _Servopos;    // position du servo moteur
    int _Servomin;    // position minimale du servo moteur
    int _Servomax;    // position maximale du servo moteur
    char _Servospeed;  // vitesse de mouvement du servo moteur
    bool _Servosens;   // sens de mouvementG du servo moteur
    char _Servopin;    // N° de la broche
};


#endif
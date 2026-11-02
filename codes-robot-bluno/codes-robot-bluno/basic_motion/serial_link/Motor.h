#ifndef Motor_h
#define Motor_h
#include "Arduino.h" 

class Motor {
  public:
    Motor();                                        // constructeur d'un moteur vide
    Motor(int pwmpin, int snspin1, int sns);         // constructeur du moteur
    void setvalue(int value);                       // definit une consigne de tension
    int getvalue();
    void setcptincr(long time, long value);         // sauvegarde la position du compteur incremental
    int  getspeed();                                // recupere la vitesse du moteur

  private:
    char _PwmPin;  // numero de broche du signal pwm
    char _SnsPin;  // numero de broche du sens
    char _Sns;     // sens d'installation du moteur
    int _Value;   // signal de commande du moteur 
    long _Tim,_LTim,_Cpt,_LCpt;  // valeurs du compteur incremental et du temps correspondant
};
  
#endif

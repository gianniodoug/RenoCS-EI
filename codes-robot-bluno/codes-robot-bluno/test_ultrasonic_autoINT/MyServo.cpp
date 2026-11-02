#include "MyServo.h"

// constructeur du servomoteur
MyServo::MyServo(int pin, int min, int max, int speed) {
	_Servomin = min;
	_Servomax = max;
	_Servospeed = speed;
	_Servosens = true;
	_Servopos = (min+max)/2 ;
	_Servopin = pin ;
	//_MonServo.attach(pin);
	//_MonServo.write(_Servopos);
}

// connexion du servomoteur
void MyServo::start(){
	_MonServo.attach(_Servopin);
}

// definit la position du servo moteur
void MyServo::setpos(int pos){
	if (pos<_Servomin) pos=_Servomin ;
	if (pos>_Servomax) pos=_Servomax ;
	_Servopos = pos ;
	_MonServo.write(_Servopos);
}

// definit la position minimale
void MyServo::setmin(int min){
	if (min<0) min=0 ;
	_Servomin = min ;
}

// definit la position maximale
void MyServo::setmax(int max){
	if (max>270) max=270 ;
	_Servomax = max ;
}

// definit la vitesse de mouvement
void MyServo::setspeed(int speed){
	_Servospeed = speed ;
}

// recupere la position minimale
int MyServo::getmin(){
	return(_Servomin)  ;
}

// recupere la position maximale
int MyServo::getmax(){
	return(_Servomax)  ;
}

// fait avancer le servo d'un pas
void MyServo::advance(){

	if (_Servosens==true) //le servo avance
    {
      if ((_Servopos+_Servospeed)>= _Servomax)
      { _Servopos=_Servomax ; _Servosens=false; }
      else
        _Servopos += _Servospeed ;
    }
  else
    {
      if ((_Servopos-_Servospeed)<= _Servomin)
      { _Servopos=_Servomin ; _Servosens=true; }
      else
        _Servopos -= _Servospeed ;
    }
  _MonServo.write(_Servopos);
}

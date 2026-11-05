#include "Arduino.h"
#include "Motor.h"

Motor::Motor() {
	_PwmPin = -1;
	_SnsPin = 0;
	_Sns = 0;
	_Value=0 ;
}

Motor::Motor(int pwmpin, int snspin, int sns) {
	pinMode(pwmpin, OUTPUT);
	pinMode(snspin, OUTPUT);
	analogWrite(pwmpin,0);
	_PwmPin = pwmpin;
	_SnsPin = snspin;
	_Sns = sns;
	_Value=0 ;
}

void Motor::setvalue(int value){
	if (_PwmPin==-1) return ;
	_Value=value ;
	analogWrite(_PwmPin,abs(_Value));
    digitalWrite(_SnsPin,_Sns?(_Value>=0?0:1):(_Value>=0?1:0)) ;
}

int Motor::getvalue(){
	return(_Value) ;
}

void Motor::setcptincr(long time, long cpt){
	 _LTim=_Tim ;
	 _LCpt=_Cpt ;
	 _Tim=time ;
	 _Cpt=cpt ;
}

int Motor::getspeed(){
	int vitesse = (_Cpt-_LCpt)*1000/(_Tim-_LTim) ;
	return(vitesse) ;
}

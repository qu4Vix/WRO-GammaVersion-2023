/*
 *
 *
 *
 */

#include "Motor.h"

Motor::Motor(byte PinEn, byte PinDir1, byte PinDir2, float kp, float kd){ // setup del motor
  _pinEn = PinEn;
  _pinDir1 = PinDir1;
  _pinDir2 = PinDir2;
  _kp = kp;
  _kd = kd;
  pinMode(PinEn, OUTPUT);
  pinMode(PinDir1, OUTPUT);
  pinMode(PinDir2, OUTPUT);
}

void Motor::Power(int pot){
  // (_pinDir1, LOW) ; (_pinDir2, HIGH) -> Backwards
  // (_pinDir1, HIGH); (_pinDir2, LOW)  -> Forward
  if (pot > 0) {
    forward = true;
    digitalWrite(_pinDir1,HIGH);
    digitalWrite(_pinDir2,LOW);
    analogWrite(_pinEn, pot);
  }
  else if (pot < 0) {
    forward = false;
    digitalWrite(_pinDir1,LOW);
    digitalWrite(_pinDir2,HIGH);
    analogWrite(_pinEn, -pot);
  } else {
    digitalWrite(_pinDir1,LOW);
    digitalWrite(_pinDir2,LOW);
    analogWrite(_pinEn,150);
  }
}

void Motor::SetSpeed(int velocidadActual, int velocidadTarget) {
  _error = velocidadTarget - velocidadActual;
  _power = constrain(_power + _error * _kp + (_error - _lastError) * _kd, -255, 255);
  _lastError = _error;
  if (velocidadTarget == 0) {
    Power(0);
  } else {
    Power(int(_power))
  ;}
}

float Motor::GetPower() {
  return _power;
}
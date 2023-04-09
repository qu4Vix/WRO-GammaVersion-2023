/*
 * Motor.h - Library for controling an engine with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "Motor.h"

Motor::Motor(byte PinEn, byte PinDir1, byte PinDir2, float kp, float kd) { // setup del motor
  _pinEn = PinEn;
  _pinDir1 = PinDir1;
  _pinDir2 = PinDir2;
  _kp = kp;
  _kd = kd;
}

void Motor::Init(){
  pinMode(_pinEn, OUTPUT);
  pinMode(_pinDir1, OUTPUT);
  pinMode(_pinDir2, OUTPUT);
}

void Motor::SetPower(int power) {
  // (_pinDir1, LOW) ; (_pinDir2, HIGH) -> Backwards
  // (_pinDir1, HIGH); (_pinDir2, LOW)  -> Forward
  if (power > 0) {
    _forward = true;
    digitalWrite(_pinDir1, HIGH);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinEn, power);
  }
  else if (power < 0) {
    _forward = false;
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, HIGH);
    analogWrite(_pinEn, -power);
  } else {
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinEn, 150);
  }
}

void Motor::SetSpeed(int actualSpeed, int targetSpeed) {
  _error = targetSpeed - actualSpeed;
  _power = constrain(_power + _error * _kp + (_error - _lastError) * _kd, -255, 255);
  _lastError = _error;
  if (targetSpeed == 0) {
    SetPower(0);
  } else {
    SetPower(int(_power));
  }
}

float Motor::GetPower() {
  return _power;
}
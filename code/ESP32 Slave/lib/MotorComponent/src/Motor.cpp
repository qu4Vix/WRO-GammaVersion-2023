/*
 * Motor.cpp - Library for controling an engine with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "Motor.h"

Motor::Motor(byte PinPWM, byte PinDir1, byte PinDir2, byte PinEnable, float kp, float kd) { // setup del motor
  _pinPWM = PinPWM;
  _pinDir1 = PinDir1;
  _pinDir2 = PinDir2;
  _pinEnable = PinEnable;
  _kp = kp;
  _kd = kd;
}

void Motor::Init() {
  pinMode(_pinPWM, OUTPUT);
  pinMode(_pinDir1, OUTPUT);
  pinMode(_pinDir2, OUTPUT);
  pinMode(_pinEnable, OUTPUT);
  digitalWrite(_pinEnable, HIGH);
}

void Motor::SetPower(int power) {
  // (_pinDir1, LOW) ; (_pinDir2, HIGH) -> Backwards
  // (_pinDir1, HIGH); (_pinDir2, LOW)  -> Forward
  if (power >= 0) {
    _forward = true;
    digitalWrite(_pinDir1, HIGH);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinPWM, power);
  }
  else if (power < 0) {
    _forward = false;
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, HIGH);
    analogWrite(_pinPWM, -power);
  } /* else {
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinPWM, 150);
  } */
}

void Motor::SetSpeed(int actualSpeed, int targetSpeed) {
  _error = targetSpeed - actualSpeed;
  _power = constrain(_power + _error * _kp + (_error - _lastError) * _kd, -255, 255);
  if (targetSpeed == 0) {
    SetPower(0);
  } else {
    SetPower(int(_power));
  }
  _lastError = _error;
}

float Motor::GetPower() {
  return _power;
}
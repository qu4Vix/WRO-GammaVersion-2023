/*
 * Servo.cpp - Library for controling a servo with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "Servo.h"

CServo::CServo(byte PinServo) {
  _pinServo = PinServo;
}

void CServo::Attach() {
  Miservo.attach(_pinServo);
}

void CServo::MoveServo(int angle) {
  _ang = map(angle, -90, 90, _servoMIN, _servoMAX);
  _ang = constrain(_ang, _servoMIN, _servoMAX);
  Miservo.write(_ang);
}

int CServo::GetAngle() {
  return _ang;
}
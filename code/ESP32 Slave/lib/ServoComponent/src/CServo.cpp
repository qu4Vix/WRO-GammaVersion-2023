/*
 * Servo.cpp - Library for controling a servo with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "CServo.h"

CServo::CServo(byte PinServo) {
  _pinServo = PinServo;
}

void CServo::BeginPWM() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
}

void CServo::Attach() {
  Miservo.setPeriodHertz(50);
  Miservo.attach(_pinServo, 1000, 2000);
}

void CServo::MoveServo(int angle) {
  _ang = map(angle, -90, 90, _servoMIN, _servoMAX);
  _ang = constrain(_ang, _servoMIN, _servoMAX);
  Miservo.write(_ang);
}

int CServo::GetAngle() {
  return _ang;
}
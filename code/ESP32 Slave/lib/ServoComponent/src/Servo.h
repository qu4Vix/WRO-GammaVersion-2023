/*
 * Servo.h - Library for controling a servo with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#ifndef Servo_h
#define Servo_h

#include <Arduino.h>
#include <ESP32Servo.h>

class CServo{  //maneja el servo
public:
  CServo(byte PinServo);
  void Attach();
  void MoveServo(int _angulo);
  int GetAngle();

private:
  Servo Miservo;
  byte _pinServo;
  int _ang = 0;
  byte _servoMIN;
  byte _servoMAX;
};

#endif
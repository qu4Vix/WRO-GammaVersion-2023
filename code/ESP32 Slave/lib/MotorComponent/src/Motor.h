/*
 *
 *
 *
 */


#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor{  
  public:
  Motor(byte PinEn, byte PinDir1, byte PinDir2, float kp, float kd);
  void Power(int pot);
  void SetSpeed(int actualSpeed, int targetSpeed);
  float GetPower();
  bool forward;

  private:
  byte _pinEn;
  byte _pinDir1;
  byte _pinDir2;
  float _power = 0;
  float _error;
  float _lastError;
  float _kp;
  float _kd;
};

#endif
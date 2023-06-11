/*
 * Motor.h - Library for controling an engine with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */


#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor{  
  public:
  Motor(byte PinPWM, byte PinDir1, byte PinDir2, byte PinEnable, float kp, float kd);
  void Init();
  void SetPower(int pot);
  void SetSpeed(int actualSpeed, int targetSpeed);
  float GetPower();

  private:
  byte _pinPWM;
  byte _pinDir1;
  byte _pinDir2;
  byte _pinEnable;
  float _power = 0;
  float _error;
  float _lastError;
  float _kp;
  float _kd;
  bool _forward;
};

#endif
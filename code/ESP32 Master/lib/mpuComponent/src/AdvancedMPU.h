/*
 * AdvancedMPU.h - Library for controling a MPU9250 and getting the overall angle with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#ifndef AdvancedMPU_h
#define AdvancedMPU_h

#include <Arduino.h>

#include <Wire.h>
#include <MPU9250.h>

class MPU {
    public:
    MPU();  // Class constructor
    void BeginWire(byte pinSDA, byte pinSCL, uint32_t frequency);   // Begin Wire 1
    void Setup();   // Set up the mpu 9265 with default settings in wire1
    void WorkOffset();  // Work out the offset of the mpu
    void UpdateAngle(); // Update the angle (Call in the loop)
    float GetAngle();   // Returns the angle
    void measureFirstMillis();

    private:
    MPU9250 _mpu;   // MPU object
    byte _pinSDA;
    byte _pinSCL;
    float _offset;
    float _angle;
    uint32_t _prev_ms_angle;
    bool _firstMillisRead;
};

#endif
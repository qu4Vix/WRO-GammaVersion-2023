/*
 * Encoder.h - Library for using an encoder with Arduino Framework.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>

class Encoder{
    public:
    Encoder(byte pinEncoder);
    void Attach(byte mode);

    private:
    static Encoder* _sEncoder;
    static void EncoderISR();
    void UpdateEncoder();
    byte _pinEncoder;
    volatile long _encoder;
    volatile long _encoderTotal;
};

#endif
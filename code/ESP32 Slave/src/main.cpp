#include <Arduino.h>
#include <OTAUpdate.h>
#include <Motor.h>
#include <CServo.h>
#include <ESP32Encoder.h>
#include "credentials.h"

// LED pins
#define pinLED_1 34
#define pinLED_2 35

// Motor Pins (PWM + 3 Digital)
#define pinPWM 14
#define pinDir1 26
#define pinDir2 27
#define pinEn 25

// Servo pin (PWM)
#define pinServo 13

// Encoder pins
#define pinEncoder_CLK 33
#define pinEncoder_DT 32

// Interchip Communication pins (UART)
#define pinTX 4
#define pinRX 2

// Battery tension reader pin
#define pinTension 36

Updater miota(81);
Motor mimotor(pinPWM, pinDir1, pinDir2, pinEn, 0, 0);
CServo miservo(pinServo);
ESP32Encoder encoder;

void actualizarBateria();
// Battery levels
// 8.4V - 3600
// 8V - 3300
// 7.6V - 3100

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Empezando");
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinTension, INPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  miota.WiFiInit();
  miota.OTAInit();

  encoder.attachFullQuad ( pinEncoder_DT, pinEncoder_CLK );
  encoder.setCount ( 0 );

  //mimotor.Init();
  //mimotor.SetPower(60);
  //miservo.BeginPWM();
  //miservo.Attach();

  
}

void loop() {
  // put your main code here, to run repeatedly:
  //actualizarBateria();
  long newPosition = encoder.getCount();
  Serial.println(newPosition);
}

void actualizarBateria() {
  uint16_t tension = analogRead(pinTension);
  if (tension >= 3300) {
    // VERDE
  } else if (tension >= 3100) {
    // AMARILLO
  } else {
    // ROJO
  }
}
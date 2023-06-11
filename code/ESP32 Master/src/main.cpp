#include <Arduino.h>
#include <AdvancedMPU.h>
#include <OTAUpdate.h>
#include <RPLidar.h>
#include "credentials.h"

// LED pins
#define pinLED_rojo 12
#define pinLED_verde 13

// Battery level LED pins
#define pinLED_batRojo 27
#define pinLED_batAmarillo 26
#define pinLED_batVerde 25

// LIDAR pins (UART2 + PWM)
#define pinLIDAR_TX 16
#define pinLIDAR_RX 17
#define pinLIDAR_motor 5

// MPU pins (I2C)
#define pinMPU_SDA 21
#define pinMPU_SCL 22

// Interchip Communication pins (UART)
#define pinTX 2
#define pinRX 4

// Button pin
#define pinBoton 33

Updater miota(80);
MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(pinBoton, INPUT_PULLUP);
  pinMode(pinLED_rojo, OUTPUT);
  pinMode(pinLED_verde, OUTPUT);
  pinMode(pinLED_batRojo, OUTPUT);
  pinMode(pinLED_batAmarillo, OUTPUT);
  pinMode(pinLED_batVerde, OUTPUT);

  miota.WiFiInit();
  miota.OTAInit();

  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  lidar.begin(lidarSerial);

  digitalWrite(pinLED_verde, HIGH);
  while(digitalRead(pinBoton));
  digitalWrite(pinLED_verde, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    
    //perform data processing here... 
    if ((angle <= 10) && (angle >= -10)) {
      Serial.print("DATA: ");
      Serial.print("distance ");
      Serial.print(distance);
      Serial.print("  angle");
      Serial.println(angle);
    }
    
  } else {
    analogWrite(pinLIDAR_motor, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(pinLIDAR_motor, 255);
       delay(1000);
    }
  }
}
#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include "credentials.h"
#include "pinAssignments.h"

#define ENABLE_WIFI true

#if ENABLE_WIFI == true
#include <OTAUpdate.h>
#include <Telemetry.h>

IPAddress receiversIP(192, 168, 144, 14);
uint16_t receiversPort = 4210;
uint16_t udpPort = 1234;
uint16_t otaPort = 1235;

Updater miota(otaPort);
TelemetryManager telemetry(receiversIP, receiversPort);
#endif

uint32_t encoderMeasurement;

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);

void setSpeed(int speed);
void setSteering(int angle);
void receiveData();
void manageTension(uint8_t tension);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  setPinModes();

  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  #if ENABLE_WIFI == true
    miota.WiFiInit();
    miota.SetStaticIP(250);
    miota.OTAInit();

    telemetry.StartUDP(udpPort);
  #endif

  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  lidar.begin(lidarSerial);

  digitalWrite(pinLED_verde, HIGH);
  while (digitalRead(pinBoton));
  digitalWrite(pinLED_verde, LOW);

  setSpeed(40);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (commSerial.available())
  {
    receiveData();
  }
}

void setSpeed(int speed) {
  commSerial.write(1);
  speed = constrain(speed, -100, 100);
  uint8_t _speed = (abs(speed) << 1) | ((speed >= 0) ? 0 : 1);
  commSerial.write(speed);
}

void setSteering(int angle) {
  commSerial.write(2);
  angle = constrain(angle, -90, 90);
  uint8_t _angle = map(angle, -90, 90, 0, 180);
  commSerial.write(_angle);
}

void receiveData() {
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  telemetry.SendString(String(firstByte));
  if (firstByte == 7) {
    uint8_t byte[4];
    commSerial.readBytes(byte, 4);
    for (uint8_t iteration; iteration < 4; iteration++) {
      encoderMeasurement = encoderMeasurement | byte[iteration] << (8*iteration);
    }
  } else if (firstByte == 6) {
    uint8_t tensionValue;
    commSerial.readBytes(&tensionValue, 1);
    manageTension(tensionValue);
  }
}

void manageTension(uint8_t tension) {
  if (tension == 1) {
    digitalWrite(pinLED_batAmarillo, LOW);
    digitalWrite(pinLED_batRojo, LOW);
    digitalWrite(pinLED_batVerde, HIGH);
  } else if (tension == 2) {
    digitalWrite(pinLED_batVerde, LOW);
    digitalWrite(pinLED_batRojo, LOW);
    digitalWrite(pinLED_batAmarillo, HIGH);
  } else if (tension == 3) {
    digitalWrite(pinLED_batVerde, LOW);
    digitalWrite(pinLED_batAmarillo, LOW);
    digitalWrite(pinLED_batRojo, HIGH);
  }
}
#include <Arduino.h>
#include <Motor.h>
#include <CServo.h>
#include <Encoder.h>
#include <Pixy2.h>
#include "credentials.h"
#include "pinAssignments.h"

#define ENABLE_WIFI fasle
#define ROUND_NUMBER 2

#if ENABLE_WIFI == true
#include <OTAUpdate.h>
Updater miota(80);
#endif

hw_timer_t* timerHandler;

HardwareSerial commSerial(1);
Motor mimotor(pinPWM, pinDir1, pinDir2, pinEn, 2, 1);
CServo miservo(pinServo);
Encoder miencoder(pinEncoder_DT);

#if ROUND_NUMBER == 2
Pixy2 pixy;
#endif

volatile int speed;
int objectiveSpeed;

void IRAM_ATTR onTimer();

void receiveData();
void sendEncoder(uint32_t encoder);
void sendTension(uint8_t batteryLevel);
void actualizarBateria();
// Battery levels
// 8.4V - 3600
// 8V - 3300
// 7.6V - 3100

#if ROUND_NUMBER == 2
void sendCamera(uint8_t signature, uint16_t x, uint16_t y);
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Empezando");
  
  setPinModes();

  commSerial.begin(1000000, SERIAL_8N1, pinTX, pinRX);

  #if ENABLE_WIFI == true
  miota.WiFiInit();
  miota.SetStaticIP(251);
  miota.OTAInit();
  #endif
  
  Serial.print("Time: ");
  delay(5000);

  timerHandler = timerBegin(0, 80, true);
  timerAttachInterrupt(timerHandler, &onTimer, false);
  timerAlarmWrite(timerHandler, 32000, true);

  #if ROUND_NUMBER == 2
  pixy.init();
  #endif

  miencoder.Attach(CHANGE);
  mimotor.Init();
  //mimotor.SetPower(40);
  miservo.BeginPWM();
  miservo.Attach();
  miservo.MoveServo(0);
  timerAlarmEnable(timerHandler);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (commSerial.available())
  {
    receiveData();
  }

  static uint32_t prev_ms_speed;
  if (millis() > prev_ms_speed) {
    mimotor.SetSpeed(speed, objectiveSpeed);
    prev_ms_speed = millis() + 32;
  }

  static uint32_t prev_ms_bat = millis();
  if (millis() > prev_ms_bat) {
    actualizarBateria();
    prev_ms_bat = millis() + 500;
  }

  static uint32_t prev_ms_encoder = millis();
  if (millis() > prev_ms_encoder) {
    sendEncoder(miencoder.GetEncoder());
    prev_ms_encoder = millis() + 32;
  }

  #if ROUND_NUMBER == 2

  static uint32_t prev_ms_camera = millis();
  if (millis() > prev_ms_camera) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
      int tamano = 0;
      int mayor = -1;
      for (uint8_t index = 0; index < pixy.ccc.numBlocks; index++) {
        if (pixy.ccc.blocks[index].m_height > tamano) {
          mayor = index;
          tamano = pixy.ccc.blocks[index].m_height;
        }
      }
    }
    sendCamera(pixy.ccc.blocks[0].m_signature, pixy.ccc.blocks[0].m_x, pixy.ccc.blocks[0].m_y);
    prev_ms_camera = millis() + 100;
  }
  #endif
}

void IRAM_ATTR onTimer() {
  speed = miencoder.GetEncoderInterval();
}

void sendEncoder(uint32_t encoder) {
  uint8_t encoderBuffer[4];
  for (uint8_t i; i<4; i++) {
    encoderBuffer[i] = ((encoder>>(8*i)) & 0xff);
  }
  commSerial.write(7);
  commSerial.write(encoderBuffer, 4);
}

void sendTension(uint8_t batteryLevel) {
  commSerial.write(6);
  commSerial.write(batteryLevel);
}

#if ROUND_NUMBER == 2
void sendCamera(uint8_t signature, uint16_t x, uint16_t y) {
  uint8_t _x = uint8_t(x/2);
  uint8_t _y = uint8_t(y/2);
  commSerial.write(5);
  commSerial.write(signature);
  commSerial.write(_x);
  commSerial.write(_y);
}
#endif

void receiveData() {
  uint8_t firstByte;
  commSerial.readBytes(&firstByte, 1);
  if (firstByte == 1)
  {
    uint8_t _velocity;
    commSerial.readBytes(&_velocity, 1);
    uint8_t _speed = (_velocity >> 1)<<1;
    int speed = (_velocity - _speed) ? -_speed : _speed;
    objectiveSpeed = speed;
  } else 
  if (firstByte == 2)
  {
    uint8_t _angleByte;
    commSerial.readBytes(&_angleByte, 1);
    int _angle = _angleByte - 90;
    miservo.MoveServo(_angle);
  }
}

void actualizarBateria() {
  uint16_t tension = analogRead(pinTension);
  if (tension >= 3300) {
    // HIGH LEVEL
    sendTension(1);
  } else if (tension >= 3100) {
    // MEDIUM LEVEL
    sendTension(2);
  } else {
    // LOW LEVEL
    sendTension(3);
  }
}
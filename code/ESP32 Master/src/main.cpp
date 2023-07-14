#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include "credentials.h"
#include "pinAssignments.h"

#define ENABLE_WIFI false

#if ENABLE_WIFI == true
#include <OTAUpdate.h>
#include <Telemetry.h>

IPAddress receiversIP(192, 168, 144, 14);
uint16_t receiversPort = 4210;
uint16_t udpPort = 1234;
uint16_t otaPort = 80;

Updater miota(otaPort);
TelemetryManager telemetry(receiversIP, receiversPort);
#endif

#define servoKP 2.5
#define servoKD 10
int prev_setAngle;
int actual_directionError;
int prev_directionError;
int objectiveDirection;

enum e {
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando
};
uint8_t estado = e::Recto;

uint8_t giros = 1;
int8_t turnSense;

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

float lidarDistance = 3000;
float lidarAngle;
float distances[360];

bool reading = false;
bool writing = false;

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;

int directionError(int bearing, int target);

void setSpeed(int speed);
void setSteering(int angle);
void receiveData();
void manageTension(uint8_t tension);

uint16_t getIndex(float angle);
// Angle from 0 to 359
float readDistance(uint16_t angle);
// Create code for task1
void Task1Code(void * pvParameters);

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
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  // detected...
  lidar.startScan();

  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
    Task1Code,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    1);
  delay(500);
  
  // start motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);
  /*
  digitalWrite(pinLED_verde, HIGH);
  while (digitalRead(pinBoton)) {
    while (commSerial.available())
    {
      receiveData();
    }
  }
  digitalWrite(pinLED_verde, LOW);*/
  delay(500);

  setSpeed(20);
  mimpu.measureFirstMillis();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (commSerial.available())
  {
    receiveData();
  }

  mimpu.UpdateAngle();

  static uint32_t prev_ms_direction = millis();
  if (millis() > prev_ms_direction) {
    actual_directionError = constrain(directionError(mimpu.GetAngle(), objectiveDirection), -127, 127);
    int _setAngle = servoKP * actual_directionError + servoKD * (actual_directionError - prev_directionError);
    if(_setAngle != prev_setAngle) {
      setSteering(_setAngle);
      prev_setAngle = _setAngle;
    }
    prev_directionError = actual_directionError;
    prev_ms_direction = millis() + 20;
  }

  if (IS_OK(lidar.waitPoint(100))) {
    float lidarDistance_new = lidar.getCurrentPoint().distance; //distance value in mm unit
    if (lidarDistance_new) {
      lidarDistance = lidarDistance_new;
      lidarAngle    = lidar.getCurrentPoint().angle; //anglue value in degree
      if (lidarAngle > 180) lidarAngle -= 360;
      bool startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    }
    
    //perform data processing here...
  }

  switch (estado)
  {
  case e::Recto:
    digitalWrite(pinLED_rojo, HIGH);
    if (giros == 9) {
      setSpeed(0);
    }
    if (abs(lidarAngle) < 1) {
      if (lidarDistance <= 1200) {
        estado = e::DecidiendoGiro;
        digitalWrite(pinLED_rojo, LOW);
      }
    }
    break;
  case e::DecidiendoGiro:
    if (abs(90 - lidarAngle) < 1) { // lidar mide + derecha - izquierda
      if (lidarDistance >= 1000) {
        digitalWrite(pinLED_verde, HIGH);
        turnSense = -1;
        prev_encoderMeasurement = encoderMeasurement;
        estado = e::PreGiro;
      }
    } else if (abs(90 + lidarAngle) < 1) {
      if (lidarDistance >= 1000) {
        digitalWrite(pinLED_verde, HIGH);
        turnSense = 1;
        prev_encoderMeasurement = encoderMeasurement;
        estado = e::PreGiro;
      }
    }
    break;
  case e::PreGiro:
    if (encoderMeasurement - prev_encoderMeasurement > 30) {
      objectiveDirection += 90*turnSense;
      estado = e::Girando;
    }
    break;
  case e::Girando:
    if (abs(90 * giros - mimpu.GetAngle() * turnSense) < 10) {
      giros++;
      estado = e::Recto;
      digitalWrite(pinLED_verde, LOW);
    }
    break;
  }
}

int directionError(int bearing, int target) {
  int error = target - bearing;
  return error;
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
  if (firstByte == 7) {
    uint8_t bytes[4];
    commSerial.readBytes(bytes, 4);
    encoderMeasurement = 0;
    for (uint8_t iteration; iteration < 4; iteration++) {
      encoderMeasurement = encoderMeasurement | bytes[iteration] << (8*iteration);
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


uint16_t getIndex(float angle) {
  if (angle >= 359.5) return 0;
  float error = angle - uint16_t(angle);
  if (error < 0.5) {
    return uint16_t(angle);
  } else {
    return uint16_t(angle + 1);
  }
}

// Angle from 0 to 359
float readDistance(uint16_t angle) {
  while (writing);
  reading = true;
  float distanceMeasure = distances[angle];
  reading = false;
  return distanceMeasure;
}

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  for (;;) {
    while (reading);
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      //bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan

      writing = true;
      //distances[uint16_t(angle)] = distance;
      uint16_t index = getIndex(angle);
      distances[index] = distance;
      writing = false;
    }
  }
}
#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include "credentials.h"
#include "pinAssignments.h"
#include "TurnSense.h"

#define ENABLE_WIFI false

#define telemetriaTX 19
#define telemetriaRX 23

#if ENABLE_WIFI == true
#include <OTAUpdate.h>
#include <Telemetry.h>

IPAddress receiversIP(192, 168, 0, 102);
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
float objectiveDirection;

#define encodersPerCM 1

enum e {
  Inicio,
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando,
  Final
};
uint8_t estado = e::Recto;

uint8_t giros = 1;
uint8_t tramo = 0;
int8_t turnSense = 1;

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

uint32_t prev_ms_tele = 0;

float lidarAngle;
uint16_t distancesArray[2][360];
volatile bool arrayLecture = false;

double xPosition = 0;
double yPosition = 0;

#define positionKP 20
#define positionKD 5
int objectivePosition = 0;
int positionError;
int prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;
HardwareSerial teleSerial(0);

int directionError(int bearing, int target);

void setSpeed(int speed);
void setSteering(int angle);
void receiveData();
void manageTension(uint8_t tension);

uint16_t getIndex(float angle);
// Angle from 0 to 359
uint16_t readDistance(uint16_t angle);
// Create code for task1
void LidarTaskCode(void * pvParameters);

void iteratePosition();
void turn();        // turn
void setXcoord();   // set the coordinate x axis
void setYcoord();   // set the coordinate y axis
void decideTurn();  // detect the sense of turn
void checkTurn();   // check wether you have to turn or not

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);

  setPinModes();

  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  #if ENABLE_WIFI == true
    miota.WiFiInit();
    miota.SetStaticIP(250);
    miota.OTAInit();

    telemetry.StartUDP(udpPort);
    digitalWrite(pinLED_rojo, HIGH);
  #endif
  
  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  lidar.begin(lidarSerial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  //Serial.println("info: " + String(health.status) +", " + String(health.error_code));

  // detected...
  lidar.startScan();

  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
    LidarTaskCode,
    "Task1",
    100000,
    NULL,
    10,
    &Task1,
    0);
  delay(500);
  digitalWrite(pinLED_rojo, LOW);
  // start motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  setYcoord();

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

  static uint32_t prev_ms_position = millis();
  if (millis() > prev_ms_position) {
    if (encoderMeasurement != prev_encoderMeasurement) {
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) / encodersPerCM;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) / encodersPerCM;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition = dx * turnSense;
      yPosition += dy;
      iteratePosition();
    }
    prev_ms_position = millis() + 32;
  }

  static uint32_t prev_ms_turn = millis();
  if (millis() > prev_ms_turn) {
    checkTurn();
    prev_ms_turn = millis() + 50;
  }

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

  switch (estado)
  {
  case e::Inicio:
    if (yPosition >= 260) {
      decideTurn();
      setXcoord();
      estado = e::Recto;
    }
  break;
  case e::Recto:
    digitalWrite(pinLED_rojo, LOW);
    digitalWrite(pinLED_verde, LOW);
    if (giros == 13) {
      estado = e::Final;
    }
  break;
  case e::Final:
    setSpeed(0);
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
uint16_t readDistance(uint16_t angle) {
  uint16_t distanceMeasure = distancesArray[arrayLecture][angle];
  if ((distanceMeasure < 3100) && (distanceMeasure > 100)){
    return distanceMeasure;
  } else return 0;
}

// Create code for the task which manages the lidar
void LidarTaskCode(void * pvParameters) {
  for (;;) {
    vTaskDelay(1);
    if (IS_OK(lidar.waitPoint())) {
      // record data
      uint16_t distance = uint16_t(lidar.getCurrentPoint().distance); //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //angle value in degrees
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
      byte quality = lidar.getCurrentPoint().quality;

      // when scan completed switch array
      if (startBit) {
        Serial.println("Startbit");
        arrayLecture = !arrayLecture;
        memset(distancesArray[!arrayLecture], 0, 360);
      }

      // obtain the index associated with the angle and store in the array
      uint16_t index = getIndex(angle);
      distancesArray[!arrayLecture][index] = distance;
      if ((angle < 0.5) || (angle >= 359.5)) {
        Serial.println("dist: "+ String(distance) + " quality: " + String(quality));
      }
    }
  }
}

void iteratePosition() {
  prev_positionError = positionError;
  if (fixXposition) {
    positionError = directionError(xPosition, objectivePosition);
    objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90);
  } else {
    positionError = directionError(yPosition, objectivePosition);
    objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90);
  }
  if (fixInverted) objectiveDirection = -objectiveDirection;
}

void turn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    objectivePosition = 275;
    fixInverted = false;
    tramo = 1;
    break;
  
  case -2:
    objectivePosition = 275;
    fixInverted = false;
    tramo = 2;
    break;

  case -3:
    objectivePosition = 25;
    fixInverted = true;
    tramo = 3;
    break;

  case -4:
    objectivePosition = 25;
    fixInverted = true;
    tramo = 0;
    break;
  
  case 1:
    objectivePosition = 275;
    fixInverted = true;
    tramo = 1;
    break;
  
  case 2:
    objectivePosition = 25;
    fixInverted = false;
    tramo = 2;
    break;

  case 3:
    objectivePosition = 25;
    fixInverted = false;
    tramo = 3;
    break;

  case 4:
    objectivePosition = 275;
    fixInverted = true;
    tramo = 0;
    break;
  }
  giros++;
  fixXposition = !fixXposition;
}

void setXcoord(uint16_t i) {
  xPosition = i;
}

void setYcoord(uint16_t f) {
  yPosition = 300 - f;
}

void checkTurn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    if (yPosition >= 275) turn();
    break;
  
  case -2:
    if (xPosition >= 275) turn();
    break;

  case -3:
    if (yPosition <= 25) turn();
    break;

  case -4:
    if (xPosition <= 25) turn();
    break;

  case 1:
    if (yPosition >= 275) turn();
    break;
  
  case 2:
    if (xPosition <= 25) turn();
    break;

  case 3:
    if (yPosition <= 25) turn();
    break;

  case 4:
    if (xPosition >= 275) turn();
    break;
  }
}
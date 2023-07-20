#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include "credentials.h"
#include "pinAssignments.h"
#include "TurnSense.h"

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
//float distances[360];
uint16_t distancesArray[2][360];
volatile bool arrayLecture = false;

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
uint16_t readDistance(uint16_t angle);
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
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  //Serial.println("info: " + String(health.status) +", " + String(health.error_code));

  // detected...
  lidar.startScan();

  // Asign task 1 to core 0
  xTaskCreatePinnedToCore(
    Task1Code,
    "Task1",
    100000,
    NULL,
    10,
    &Task1,
    0);
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

  //setSpeed(20);
  mimpu.measureFirstMillis();
}

void loop() {
  Serial.println(readDistance(0));
  delay(20);
}

void eloop() {
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
  
  // if (IS_OK(lidar.waitPoint(100))) {
  //   float lidarDistance_new = lidar.getCurrentPoint().distance; //distance value in mm unit
  //   if (lidarDistance_new) {
  //     lidarDistance = lidarDistance_new;
  //     lidarAngle    = lidar.getCurrentPoint().angle; //anglue value in degree
  //     if (lidarAngle > 180) lidarAngle -= 360; // lidar mide + derecha - izquierda
  //     bool startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
  //   }
    
  //   //perform data processing here...
  // }
  float distance0 = readDistance(0);
  switch (estado)
  {
  case e::Recto:
    digitalWrite(pinLED_rojo, LOW);
    digitalWrite(pinLED_verde, LOW);
    if (giros == 13) {
      setSpeed(0);
    } else
    if ((distance0 < 1200) && (distance0 > 0)) {
      digitalWrite(pinLED_rojo, HIGH);
      estado = e::DecidiendoGiro;
    }
    break;
  case e::DecidiendoGiro:
    if (readDistance(90) >= 1000) {
      turnSense = -1;
      objectiveDirection -= 90;
      estado = e::Girando;
    } else
    if (readDistance(270) >= 1000) {
      turnSense = 1;
      objectiveDirection += 90;
      estado = e::Girando;
    }
    break;
  case e::Girando:
    digitalWrite(pinLED_verde, HIGH);
    if (abs(90 * giros - mimpu.GetAngle() * turnSense) < 10) {
      giros++;
      estado = e::Recto;
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
uint16_t readDistance(uint16_t angle) {
  //while (writing);
  //reading = true;
  uint16_t distanceMeasure = distancesArray[arrayLecture][angle];
  //reading = false;
  return distanceMeasure;
}

// Create code for task1
void Task1Code(void * pvParameters) {
  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  for (;;) {
    //while (reading);
    vTaskDelay(1);
    //Serial.println(xPortGetCoreID());
    if (IS_OK(lidar.waitPoint())) {
      
      uint16_t distance = uint16_t(lidar.getCurrentPoint().distance); //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      byte quality = lidar.getCurrentPoint().quality;

      if (startBit) {
        Serial.println("Startbit: " + String(startBit));
        arrayLecture = !arrayLecture;
        memset(distancesArray[!arrayLecture], 0, 360);
      }

      //writing = true;
      uint16_t index = getIndex(angle);
      distancesArray[!arrayLecture][index] = distance;
      if ((angle<0.5) || (angle>=359.5)) {
        Serial.println("dist: "+String(distance) + " quality: " + String(quality));
      }
      //writing = false;
    }
  }
}
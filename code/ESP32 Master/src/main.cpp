#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include "credentials.h"
#include "pinAssignments.h"

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

uint8_t bateria;

bool firma1Detectada = true;
uint8_t firma1X = 18;
uint8_t firma1Y = 19;
bool firma2Detectada = true;
uint8_t firma2X = 20;
uint8_t firma2Y = 21;

#define MMperEncoder 1.41

enum e {
  Inicio,
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando,
  Final
};
uint8_t estado = e::Inicio;

uint8_t giros = 1;
uint8_t tramo = 0;
int8_t turnSense = 0;

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

uint32_t prev_ms_tele = 0;

float lidarAngle;
uint16_t distancesArray[2][360];
volatile bool arrayLecture = false;

uint16_t distances[360];
static uint16_t distancesMillis[360];

double xPosition = 0;
double yPosition = 0;

#define positionKP 0.1
#define positionKD 1
int objectivePosition = 0;
int positionError;
int prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

int lidar0;

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

void enviarDato(byte* pointer, int8_t size);

void iteratePosition();
void turn();        // turn
void setXcoord(uint16_t i);   // set the coordinate x axis
void setYcoord(uint16_t f);   // set the coordinate y axis
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

  while (readDistance(0) == 0)
  {
    digitalWrite(pinLED_batAmarillo, HIGH);
  }
  digitalWrite(pinLED_batAmarillo, LOW);
  digitalWrite(pinLED_batVerde, HIGH);
  setYcoord(readDistance(0));
  lidar0=readDistance(0);
  digitalWrite(pinLED_batVerde, HIGH);

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

  setSpeed(5);
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
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition -= dx; // x -> + derecha - izquierda
      yPosition += dy;
      iteratePosition();
    }
    prev_ms_position = millis() + 32;
  }

  if (millis() > prev_ms_tele+100)
  {

    /*FORMATO TELEMETRIA
    |inicioTX            |TipoPaquete|Datos|
      0xAA,0xAA,0xAA,0xAA,0x--,0x--...0x--
    */
   /*ENVIAMOS PAQUETE TIPO 4 DISTANCIAS*/
    for(int i = 0; i<4; i++){
      teleSerial.write(0xAA);
    }
    teleSerial.write(04);
    uint16_t zi=0;
    while (zi < 360)
    {
        teleSerial.write(distances[zi]>>8);
        teleSerial.write(distances[zi]&0x00ff);
        zi++;
    }
    /*/ENVIAMOS PAQUETE TIPO 3 CALIDAD MEDIDA/
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(03);
    uint16_t pi=0;
    while (pi < 360){
        teleSerial.write(distancesArray[1][pi]);
        pi++;
    }*/

    /*ENVIAMOS PAQUETE TIPO 5 INFORMACION GENERAL*/
                /*
            --Posicion x 8 bytes
            --Posición y 8 bytes
            --Posición x Objetivo 8 bytes
            --Posición y Objetivo 8 bytes
            --Encoder 32 uint32
            --Estado 8bits  uint
            --batería 8bits uint
            --Ángulo 16 float
            --Angulo Objetivo 16 float
            --Cámara firma1 Detectada 1 byte
            --Cámara firma1 x 8 bits
            --Cámara firma1 y 8 bits
            --Cámara firma2 Detectada 1byte
            --Cámara firma2 x 8bits
            --Cámara firma2 y 8bits
            
            |XXXX|YYYY|MMMM|NNNN|QQQQ|W|E|RRRR|TTTT|U|I|O|A|S|D
             0000 0000 0111 1111 1112 2 2 2222 2223 3 3 3 3 3 3
             1234 5678 9012 3456 7890 1 2 3456 7890 1 2 3 4 5 6
            */
    for(int i = 0; i<4; i++){   //Enviamos la cabecera de inicio de paquete
      teleSerial.write(0xAA);
    }
    teleSerial.write(05);
    long posXLong = xPosition;
    long posYLong = yPosition;
    long posXObjLong = objectivePosition;
    long posYObjLong = (turnSense==-1)?1:(turnSense==1)?2:0;
    long anguloLong = mimpu.GetAngle();
    long anguloObjLong = objectiveDirection;
    enviarDato((byte*)&posXLong,sizeof(posXLong));
    enviarDato((byte*)&posYLong,sizeof(posYLong));
    enviarDato((byte*)&posXObjLong,sizeof(posXObjLong));
    enviarDato((byte*)&posYObjLong,sizeof(posYObjLong));
    enviarDato((byte*)&encoderMeasurement,sizeof(encoderMeasurement));
    enviarDato((byte*)&estado,sizeof(estado));
    enviarDato((byte*)&bateria,sizeof(bateria));
    enviarDato((byte*)&anguloLong,sizeof(anguloLong));
    enviarDato((byte*)&anguloObjLong,sizeof(anguloObjLong));
    enviarDato((byte*)&firma1Detectada,sizeof(firma1Detectada));
    enviarDato((byte*)&firma1X,sizeof(firma1X));
    enviarDato((byte*)&firma1Y,sizeof(firma1Y));
    enviarDato((byte*)&firma2Detectada,sizeof(firma2Detectada));
    enviarDato((byte*)&firma2X,sizeof(firma2X));
    enviarDato((byte*)&firma2Y,sizeof(firma2Y));    

    prev_ms_tele = millis();
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
    if (yPosition >= 2500) {
      decideTurn();
      setXcoord(readDistance(270));
      objectivePosition = xPosition;
      estado = e::Recto;
    }
  break;
  case e::Recto:
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
  commSerial.write(_speed);
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
    bateria = tensionValue;
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
  int index = -5;
  int validIndex = index;
  int validMeasures[10];

  while (index < 5) {
    // work out the resultant index for the distances array
    int resAngle = angle + index;
    if (resAngle < 0) resAngle += 360;

    // check whether the measurement is non zero and new and store it into the next place of the array
    if (distances[resAngle] == 0) {}
    else if ((millis() - distancesMillis[resAngle]) < 500)
    {
      validMeasures[validIndex] = resAngle;
      validIndex++;
    }

    index++;
  }
  // search for two consecutive measurements that are similar
  for (int arrayIndex = 1; arrayIndex < validIndex; arrayIndex++) {
    if (abs(distances[validMeasures[arrayIndex]] - distances[validMeasures[arrayIndex - 1]]) < 50) {
      return distances[validMeasures[arrayIndex]];
    }
  }
  // if the search fails return 0
  return 0;
}

// Create code for the task which manages the lidar
void LidarTaskCode(void * pvParameters) {
  for (;;) {
    vTaskDelay(1);
    if (IS_OK(lidar.waitPoint())) {
      // record data
      uint16_t distance = uint16_t(lidar.getCurrentPoint().distance); //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //angle value in degrees

      // obtain the index associated with the angle and store in the array
      uint16_t index = getIndex(angle);

      if (distance > 100 && distance < 3000)
      { 
        //bool  startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
        //byte quality = lidar.getCurrentPoint().quality;

        distances[index] = distance;
        distancesMillis[index] = millis();
      } else {
        distances[index] = 0;
        distancesMillis[index] = 0;
      }
    }
  }
}

void iteratePosition() {
  prev_positionError = positionError;
  if (fixXposition) {
    positionError = directionError(xPosition, objectivePosition);
  } else {
    positionError = directionError(yPosition, objectivePosition);
  }
  objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90);
  if (fixInverted) objectiveDirection = -objectiveDirection;
  objectiveDirection += 90 * (giros-1) * turnSense;
}

void turn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    objectivePosition = 2700;
    fixInverted = false;
    tramo = 1;
    break;
  
  case -2:
    objectivePosition = 2700;
    fixInverted = false;
    tramo = 2;
    break;

  case -3:
    objectivePosition = 300;
    fixInverted = true;
    tramo = 3;
    break;

  case -4:
    objectivePosition = 300;
    fixInverted = true;
    tramo = 0;
    break;
  
  case 1:
    objectivePosition = 2700;
    fixInverted = true;
    tramo = 1;
    break;
  
  case 2:
    objectivePosition = 400;
    fixInverted = false;
    tramo = 2;
    break;

  case 3:
    objectivePosition = 400;
    fixInverted = false;
    tramo = 3;
    break;

  case 4:
    objectivePosition = 2700;
    fixInverted = true;
    tramo = 0;
    break;
  }
  giros++;
  fixXposition = !fixXposition;
  firma1Detectada = fixXposition;
}

void setXcoord(uint16_t i) {
  xPosition = i;
}

void setYcoord(uint16_t f) {
  yPosition = 3000 - f - 150 ;
}

void checkTurn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    if (yPosition >= 2700) turn();
    break;
  
  case -2:
    if (xPosition >= 2700) turn();
    break;

  case -3:
    if (yPosition <= 300) turn();
    break;

  case -4:
    if (xPosition <= 300) turn();
    break;

  case 1:
    if (yPosition >= 2700 - 200) turn();
    break;
  
  case 2:
    if (xPosition <= 300 + 200) turn();
    break;

  case 3:
    if (yPosition <= 300 + 200) turn();
    break;

  case 4:
    if (xPosition >= 2700 - 200) turn();
    break;
  }
}

void decideTurn(){
  if (readDistance(90) > readDistance(270) && readDistance(90) > 1000)
  {
    turnSense = -1;
  }
  else if (readDistance(270) > readDistance(90) && readDistance(270) > 1000)
  {
    turnSense = 1;
  }
  else turnSense = 0;
}

void enviarDato(byte* pointer, int8_t size){
  int8_t posicion = size - 1;   //recorreremos la memoria desde el de mas valor hara el de menos, ya que el 
                          //receptor espera ese orde MSB
  while(posicion >= 0 ){
    teleSerial.write(pointer[posicion]);
    posicion--;
  }
}
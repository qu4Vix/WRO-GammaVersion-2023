#include <Arduino.h>
#include <AdvancedMPU.h>
#include <RPLidar.h>
#include <melody_factory.h>
#include <melody_player.h>
#include "credentials.h"
#include "pinAssignments.h"

// Enables wifi functions when true
#define ENABLE_WIFI false

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

// Servo and direction variables

#define servoKP 2.5
#define servoKD 0
int prev_setAngle;
int actual_directionError;
int prev_directionError;
float objectiveDirection;

// battery level variable
uint8_t bateria;

// camera signatures

bool firma1Detectada = true;
uint8_t firma1X = 18;
uint8_t firma1Y = 19;
bool firma2Detectada = true;
uint8_t firma2X = 20;
uint8_t firma2Y = 21;

// telemetry (ignore)
uint8_t arrayBloques[8];

// conversion between mm and encoder counts
#define MMperEncoder 1.41

// List of possible states for the car
enum e {
  Inicio,
  Recto,
  DecidiendoGiro,
  PreGiro,
  Girando,
  Final
};
uint8_t estado = e::Inicio;

// journey variables

uint8_t giros = 0;
uint8_t tramo = 0;
int8_t turnSense = 0;

// encoder variables

uint32_t encoderMeasurement;
uint32_t prev_encoderMeasurement;

// lidar measurement variables

uint16_t distances[360];
static uint16_t distancesMillis[360];

// car position on the map (origin at the bottom-left corner)

// x position of the car (increases to the right)
double xPosition = 0;
// y position of the car (increases upwards)
double yPosition = 0;

// position PID controller variables

#define positionKP 0.1
#define positionKD 1
int objectivePosition = 0;
int positionError;
int prev_positionError;
bool fixXposition = true;
bool fixInverted = true;

// object declarations

MPU mimpu;
HardwareSerial lidarSerial(2);
RPLidar lidar;
HardwareSerial commSerial(1);
TaskHandle_t Task1;
HardwareSerial teleSerial(0);
MelodyPlayer player(pinBuzzer, HIGH);
const uint8_t nNotes = 45;
String notes1[nNotes] = 
{
  "A4", "E5", "SILENCE",    "A4", "E5", "SILENCE",    "B4", "E5", "SILENCE",    "B4", "E5", "SILENCE",
  "C4", "E5", "SILENCE",    "C4", "E5", "SILENCE",    "D4", "E5", "SILENCE",    "D4", "E5", "B4",
  "A4", "E5", "SILENCE",    "A4", "E5", "SILENCE",    "B4", "E5", "SILENCE",    "B4", "E5", "SILENCE",
  "C4", "E5", "SILENCE",    "C4", "E5", "SILENCE",    "D4", "E5", "SILENCE"
};
const uint8_t timeUnit = 625;
// create a melody
Melody melody1 = MelodyFactory.load("Cornfield chase", timeUnit, notes1, nNotes);
String notes2[9] = {"D4", "D4", "D4", "D4", "E5", "E5", "E5", "E5", "B4"};
Melody melody2 = MelodyFactory.load("Cornfield chase 2", 156, notes2, 9);
String notes3[60] =
{
  "A4", "C4", "E5", "A5",   "A4", "C4", "E5", "A5",   "A4", "C4", "E5", "A5",
  "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",
  "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",   "B4", "D4", "E5", "B5",
  "A4", "C4", "E5", "A5",   "G3", "B4", "E5", "F5",   "A4", "C4", "E5", "A5",
  "B4", "D4", "E5", "B5",   "A4", "C4", "E5", "A5",   "G3", "B4", "E5", "F5"
};
Melody melody3 = MelodyFactory.load("Cornfield chase 3", 156, notes3, 60);
String notes4[18] =
{
  "A5", "E6", "A5",   "A5", "E6", "A5",   "B5", "E6", "B5",   "B5", "E6", "B5",   "C5", "E6", "C5",   "C5", "E6", "C5"
};
Melody melody4 = MelodyFactory.load("Cornfield chase 4", 625, notes4, 18);

// calculate the error in the direction
int directionError(int bearing, int target);

// esp32 intercommunication functions

// set the car's speed
void setSpeed(int speed);
// set the angle of the servo
void setSteering(int angle);
// receive data from the serial
void receiveData();
// turn a led on depending on the tension received
void manageTension(uint8_t tension);

// LIDAR management variables

// get the index in the distances array for an angle given
uint16_t getIndex(float angle);
// Angle from 0 to 359
uint16_t readDistance(uint16_t angle);
// Create code for the task which manages the LIDAR
void LidarTaskCode(void * pvParameters);

// send a piece of data to the telemetry esp32
void enviarDato(byte* pointer, int8_t size);

// functions for the management of the car's position

void iteratePositionPID();  // invoke an iteration of the pid controller for the position
void turn();            // turn
void setXcoord(uint16_t i);   // set the coordinate x axis
void setYcoord(uint16_t f);   // set the coordinate y axis
void decideTurn();  // detect the sense of turn
void checkTurn();   // check wether you have to turn or not

void setup() {
  // put your setup code here, to run once:

  // begin serial
  Serial.begin(115200);
  // begin telemetry serial
  teleSerial.begin(1000000, SERIAL_8N1, telemetriaRX, telemetriaTX);
  // begin esp32 intercommunication serial
  commSerial.begin(1000000, SERIAL_8N1, pinRX, pinTX);

  // set all the pin modes
  setPinModes();

  #if ENABLE_WIFI == true
    miota.WiFiInit();
    miota.SetStaticIP(250);
    miota.OTAInit();

    telemetry.StartUDP(udpPort);
  #endif
  player.play(melody1);
      player.play(melody2);
      player.play(melody3);
      player.play(melody4);
  // configure the mpu
  mimpu.BeginWire(pinMPU_SDA, pinMPU_SCL, 400000);
  mimpu.Setup();
  mimpu.WorkOffset();

  // begin the lidar
  lidar.begin(lidarSerial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  lidar.startScan();

  // Asign lidar Task to core 0
  xTaskCreatePinnedToCore(
    LidarTaskCode,
    "Task1",
    100000,
    NULL,
    10,
    &Task1,
    0);
  delay(500);

  // start lidar's motor rotating at max allowed speed
  analogWrite(pinLIDAR_motor, 255);
  delay(500);

  // wait until y coordinate is calculated
  while (readDistance(0) == 0)
  {
    digitalWrite(pinLED_rojo, HIGH);
  }
  digitalWrite(pinLED_rojo, LOW);
  digitalWrite(pinLED_verde, HIGH);
  setYcoord(readDistance(0));
  digitalWrite(pinLED_verde, HIGH);

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

  // start driving (set a speed to the car and initialize the mpu)
  setSpeed(2);
  mimpu.measureFirstMillis();
}

void loop() {
  // put your main code here, to run repeatedly:

  // receive data from the intercommunication serial
  while (commSerial.available())
  {
    receiveData();
  }

  // update mpu's angle
  mimpu.UpdateAngle();

  // repeat position functions every 32ms
  static uint32_t prev_ms_position = millis();
  if (millis() > prev_ms_position) {
    if (encoderMeasurement != prev_encoderMeasurement) {
      // calculate the increment in position and add it
      double dy = (encoderMeasurement - prev_encoderMeasurement) * cos(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      double dx = (encoderMeasurement - prev_encoderMeasurement) * sin(mimpu.GetAngle() * (M_PI/180)) * MMperEncoder;
      prev_encoderMeasurement = encoderMeasurement;
      xPosition -= dx; // x -> + derecha - izquierda
      yPosition += dy;
      iteratePositionPID();
    }
    prev_ms_position = millis() + 32;
  }

  // send telemetry every 100ms
  static uint32_t prev_ms_tele = millis();
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
    enviarDato((byte*)&arrayBloques,sizeof(arrayBloques));
    enviarDato((byte*)&tramo,sizeof(tramo));

    prev_ms_tele = millis();
  }

  // check turn every 50ms
  static uint32_t prev_ms_turn = millis();
  if (millis() > prev_ms_turn) {
    checkTurn();
    prev_ms_turn = millis() + 50;
  }

  // repeat direction pid iterations every 20ms
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

  // state machine
  switch (estado)
  {
  case e::Inicio:
    if (yPosition >= 2500) {
      decideTurn();
      if (turnSense != 0) {
        setXcoord(readDistance(270));
        objectivePosition = xPosition;
        analogWrite(pinLIDAR_motor, 0);
        estado = e::Recto;
        setSpeed(5);
      }
    }
  break;
  case e::Recto:
    if (giros == 12) {
      estado = e::Final;
      
    }
  break;
  case e::Final:
    if (yPosition >= 1200) {
      setSpeed(0);
    }
  break;
  }
}

int directionError(int bearing, int target) {
  int error = target - bearing;
  return error;
}

void setSpeed(int speed) {
  speed = constrain(speed, -100, 100);
  uint8_t _speed = (abs(speed) << 1) | ((speed >= 0) ? 0 : 1);
  commSerial.write(1);
  commSerial.write(_speed);
}

void setSteering(int angle) {
  angle = constrain(angle, -90, 90);
  uint8_t _angle = angle + 90;
  commSerial.write(2);
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

void iteratePositionPID() {
  prev_positionError = positionError;
  if (fixXposition) {
    positionError = directionError(xPosition, objectivePosition);
  } else {
    positionError = directionError(yPosition, objectivePosition);
  }
  objectiveDirection = constrain(positionKP * positionError + positionKD * (positionError - prev_positionError), -90, 90);
  if (fixInverted) objectiveDirection = -objectiveDirection;
  objectiveDirection += 90 * giros * turnSense;
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
  yPosition = 3000 - f - 150;
}

void checkTurn() {
  switch ((tramo+1) * turnSense)
  {
  case -1:
    if (yPosition >= 2700 - 200) turn();
    break;
  
  case -2:
    if (xPosition >= 2700 - 200) turn();
    break;

  case -3:
    if (yPosition <= 300 + 200) turn();
    break;

  case -4:
    if (xPosition <= 300 + 300) turn();
    break;

  case 1:
    if (yPosition >= 2700 - 300) turn();
    break;
  
  case 2:
    if (xPosition <= 300 + 300) turn();
    break;

  case 3:
    if (yPosition <= 300 + 300) turn();
    break;

  case 4:
    if (xPosition >= 2700 - 300) turn();
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
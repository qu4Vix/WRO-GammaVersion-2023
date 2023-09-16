#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "credentials.h"

#define inputBufferSize 1000
#define timeout 100

HardwareSerial SerialTelem(1);
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);

const char * udpAddress = " 192.168.30.51";
const int udpPort = 5005;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

uint8_t inputBuffer[inputBufferSize];
uint32_t timeStart;
uint16_t posBuffer;

uint8_t TeleBuffer[1000];
uint16_t posTele;

uint16_t DataLenth[6] = {5,10,15,360,720,45};   //Dependiendo del tipo de paquete, éste contendrá cierta cantidad de datos
                                                //  -0 -> 5 datos     ->NA
                                                //  -1 -> 10 datos    ->NA
                                                //  -2 -> 15 datos    ->NA
                                                //  -3 -> 360 datos   ->Lidar Quality
                                                //  -4 -> 720 datos   ->Lidar Distances
                                                //  -5 -> 45 datos    ->Información general

  /* TIPO 5 INFORMACION GENERAL
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
            --ArrayTramo      8 bytes
            --tramo           1 byte
            
            |XXXX|YYYY|MMMM|NNNN|QQQQ|W|E|RRRR|TTTT|U|I|O|A|S|D|arrayTramo|tramo
             0000 0000 0111 1111 1112 2 2 2222 2223 3 3 3 3 3 3 33344444   4
             1234 5678 9012 3456 7890 1 2 3456 7890 1 2 3 4 5 6 78901234   5
            */
                                                

//Hagamos parpadear el led (un poco)
void blinkLed(){
  if((millis()/10) % 10 == 0 ){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  }
}

void receiveData();

void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(1000000);
  SerialTelem.begin(1000000,SERIAL_8N1,5,6); //Rx = 5, Tx = 6

  connectToWiFi(ssid, password);
  timeStart = millis();
  posBuffer = 0;
  posTele = 0;

  delay(2000);
  Serial.println("Antes");

  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Conexión establecida");
  udp.endPacket();

  Serial.println("Despues");
}

void loop(){
  if(SerialTelem.available()){
    receiveData();
  }
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          digitalWrite(LED_BUILTIN,HIGH);
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          digitalWrite(LED_BUILTIN,LOW);
          break;
    }
}

#define bytes_cabecera1 1
#define bytes_cabecera2 2
#define bytes_cabecera3 3

enum cab {
  voidCabecera,
  LIDAR,
  estado,
  encoders
};

enum RXState {
  idle,
  recibiendoInicioTX,
  recibiendoTipoPaquete,
  recibiendoDatos,
};

void receiveData() {
  static RXState rxState = RXState::idle;
  static uint8_t NumPreambulo = 0;
  static uint16_t posBufferTelem = 0;   //Inicializamos el incide que nos indica la posicion del buffer en la que tenemos que escribir
  static uint16_t datosPendientesDeRecibir = 0;    //Aqui guardaremos la cantidad de datos que está pendiente de recibir, que dependera del tipo de paquete
  uint8_t rx = SerialTelem.read();
  switch (rxState) {
    case RXState::idle :
      if(rx == 0xAA){           //Recibido inicio de pqquete
        rxState = RXState::recibiendoInicioTX;
        NumPreambulo=1;         //Ya tenemos el primero
      }
    break;
    case RXState::recibiendoInicioTX :
      if(rx == 0xAA){
        NumPreambulo++;
        if(NumPreambulo > 3){   //Hemos recibido ya los 4 0xAA que indican el inicio del paquete
          rxState = RXState::recibiendoTipoPaquete;
        }
      }else{                    //No era un inicio de paquete
        udp.beginPacket(udpAddress,udpPort);//Como ha fallado algo, vamos a enviar una tráma general en la que indiquemos que algo ha fallado
        udp.write(5);
        udp.printf("Se esperaba la cabercera de inicio pero no llego--");
                  //00000000001111111111222222222233333333334444444444
                  //01234567890123456789012345678901234567890123456789
        udp.endPacket();
        rxState = RXState::idle;
      }
    break;
    case RXState::recibiendoTipoPaquete :
      if(rx < 6){    //Por ahora unicamente tenemos 5 tipos de paquetes
        posBufferTelem = 0;
        TeleBuffer[posBufferTelem] = rx;      //Guardamos el tipo de paquete en el buffer para que esté al inicio de la trama udp que vamos a enviar
        posBufferTelem++;
        datosPendientesDeRecibir = DataLenth[rx]; //Dependiendo del tipo de paquete, tendremos que recibir una cantidad de datos
        rxState = RXState::recibiendoDatos;   //Siguiente estado
      }else{    //Hemos recibido un tipo de paquete no reconocido, y no sabremos el tamaño, por lo que lo ignoraremos
        rxState = RXState::idle;
      }    
    break;
    case RXState::recibiendoDatos :
      datosPendientesDeRecibir--;
      TeleBuffer[posBufferTelem] = rx;
      posBufferTelem++;
      if(datosPendientesDeRecibir == 0){    //Ya hemos recibido todos los datos
        udp.beginPacket(udpAddress,udpPort);//Enviamos los datos por UDP
        udp.write(TeleBuffer, posBufferTelem);  //El tamaño coincide con la posicion en la que estamos en el TeleBuffer
        udp.endPacket();
        rxState = RXState::idle;            //Volvemos al estado inicial
      }
    break;
  }



/*

  posTele = 0;
  uint8_t cabecera = SerialTelem.read();
  if(cabecera < 6){
    TeleBuffer[posTele] = cabecera;
    posTele++;

    for (posTele = 1; posTele < DataLenth[cabecera]+1; posTele++)
    {
      while (!SerialTelem.available());
      TeleBuffer[posTele] = SerialTelem.read();
    }
    
    udp.beginPacket(udpAddress,udpPort);
    udp.write(TeleBuffer, posTele);
    udp.endPacket();
  }else{
    udp.beginPacket(udpAddress,udpPort);
    udp.write(5);
    udp.printf("No ha llegado la cabecera-------------------------");
  //            00000000001111111111222222222233333333334444444444
  //            01234567890123456789012345678901234567890123456789
    udp.endPacket();
  }
*/

/*
  if (cabecera == cab::LIDAR) {
    uint8_t receivedBytes[bytes_cabecera1];
    SerialTelem.readBytes(receivedBytes, bytes_cabecera1);
    udp.beginPacket(udpAddress,udpPort);
    udp.write(receivedBytes, bytes_cabecera1);
    udp.endPacket();
  } else
  if (cabecera == cab::estado) {
    uint8_t receivedBytes[bytes_cabecera2];
    for (byte i=0; i<bytes_cabecera2;i++){
      receivedBytes[i]=Serial.read();
    }
    SerialTelem.write(receivedBytes, bytes_cabecera2);
    udp.beginPacket(udpAddress,udpPort);
    udp.write(receivedBytes, bytes_cabecera2);
    udp.endPacket();
  } else
  if (cabecera == cab::encoders) {
    uint8_t receivedBytes[bytes_cabecera3];
    for (byte i=0; i<bytes_cabecera3;i++){
      receivedBytes[i]=Serial.read();
    }
    udp.beginPacket(udpAddress,udpPort);
    udp.write(receivedBytes, bytes_cabecera3);
    udp.endPacket();
  }*/
}
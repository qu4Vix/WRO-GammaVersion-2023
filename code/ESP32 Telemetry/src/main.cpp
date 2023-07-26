#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define inputBufferSize 1000
#define timeout 100

HardwareSerial SerialTelem(1);
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);

const char * udpAddress = "192.168.137.1";
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

uint16_t DataLenth[5] = {0,1,2,3,720};

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

  connectToWiFi("DESKTOP-T4CK6AC 1519", "53J542q[");
  timeStart = millis();
  posBuffer = 0;
  posTele = 0;
}

void loop(){
  if(Serial.available()){
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

void receiveData() {
  uint8_t cabecera = Serial.read();
  TeleBuffer[posTele] = cabecera;
  posTele++;

  for (posTele = 0; posTele < DataLenth[cabecera]; posTele++)
  {
    TeleBuffer[posTele] = Serial.read();
  }
  
  udp.beginPacket(udpAddress,udpPort);
  udp.write(TeleBuffer, posTele);
  udp.endPacket();

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
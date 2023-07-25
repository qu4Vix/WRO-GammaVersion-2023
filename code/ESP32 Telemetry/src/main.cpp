#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define inputBufferSize 1000
#define timeout 100

HardwareSerial SerialTelem(1);
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);

const char * udpAddress = "192.168.1.35";
const int udpPort = 5005;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

uint8_t inputBuffer[inputBufferSize];
uint32_t timeStart;
uint16_t posBuffer;

//Hagamos parpadear el led (un poco)
void blinkLed(){
  if((millis()/10) % 10 == 0 ){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  }
}

void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(115200);
  SerialTelem.begin(1000000,SERIAL_8N1,5,6); //Rx = 5, Tx = 6

  connectToWiFi("xxx", "xxx");
  timeStart = millis();
  posBuffer = 0;
}

void loop(){
  if(SerialTelem.available()){
    timeStart=millis();
    while (millis() - timeStart < timeout){
      blinkLed();
      if(SerialTelem.available()){
        if(posBuffer < inputBufferSize){
          inputBuffer[posBuffer] = SerialTelem.read();
          posBuffer++;
        }
      }
    }
    digitalWrite(LED_BUILTIN,HIGH);
    if(connected){
      //Send a packet
      udp.beginPacket(udpAddress,udpPort);
      udp.write(inputBuffer,posBuffer);
      udp.endPacket();
    }
    posBuffer = 0;
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
          digitalWrite(LED_BUILTIN,HIGH);
          break;
    }
}
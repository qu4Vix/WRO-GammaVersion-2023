/*
 * OTAUpdate.cpp - Library to upload de code over the air with an ESP32.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#include "OTAUpdate.h"

extern const char* ssid;
extern const char* password;

Updater::Updater(uint16_t port)
: _server(port)
{
    _port = port;

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hi! I am an ESP32.");
    });

    _server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404);
    });
}

void Updater::WiFiInit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void Updater::OTAInit() {
    AsyncElegantOTA.begin(&_server);    // Start ElegantOTA
    _server.begin();                    // Start the web server
    Serial.println("HTTP server started");
}
/*
 * OTAUpdate.h - Library to upload de code over the air with an ESP32.
 *
 * Created by the Gamma Version Team, 2023
 * 
 * 
 */

#ifndef OTAUpdate_h
#define OTAUpdate_h

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

class Updater {
    public:
    Updater(uint16_t port);
    void WiFiInit();
    void OTAInit();

    private:
    AsyncWebServer _server;
    uint16_t _port;
};

#endif
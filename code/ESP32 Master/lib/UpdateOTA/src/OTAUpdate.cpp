/*
 * OTAUpdate.cpp - Library to upload de code over the air with an ESP32.
 *
 * Copyright 2023 Gamma Version
 * 
 * Copyright (C) 2014-2021 Espressif Systems
 * Copyright (C) 2015-2019 Peter Andersson
 * Copyright (C) 2015-2016 Ivan Grokhotkov
 *
 * This file is part of the OTAUpdate library, which is part of WRO-GammaVersion-2023 <https://github.com/qu4Vix/WRO-GammaVersion-2023>.
 *
 * OTAUpdate is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OTAUpdate is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OTAUpdate.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "OTAUpdate.h"
#include <AsyncElegantOTA.h>

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
        Serial.println(WiFi.status());
        delay(200);
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

void Updater::SetStaticIP(uint8_t fourth_octet) {
    const IPAddress staticIP(192, 168, 0, fourth_octet);
    const IPAddress gateway(192, 168, 0, 1);
    const IPAddress subnet(255, 255, 255, 0);
    while (!WiFi.config(staticIP, gateway, subnet)) {
        delay(100);
    }
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

AsyncWebServer Updater::GetServer() {
    return _server;
}
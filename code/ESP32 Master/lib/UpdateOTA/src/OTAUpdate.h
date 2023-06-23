/*
 *
 * OTAUpdate.h - Library to upload de code over the air with an ESP32.
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

#ifndef OTAUpdate_h
#define OTAUpdate_h

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

extern const char* ssid;        // To be defined externally (Recommendable in a separate file)
extern const char* password;    // To be defined externally (Recommendable in a separate file)

class Updater {
    public:
    Updater(uint16_t port);     // Class constructor, creates the Server object in the port established; additionally the routes are created
    void WiFiInit();            // Begins the WiFi with the ssid and the password externally defined
    void OTAInit();             // Begins the OTA and the Server
    bool SetStaticIP(uint8_t fourth_octet);         // Sets a static IP in order to make upload easier
    AsyncWebServer GetServer(); // Returns the Server object (In order to assign more routes)

    private:
    AsyncWebServer _server;
    uint16_t _port;
};

#endif
/*
 *
 * This is an example sketch for the OTAUpdate library.
 * It demonstrates how to use the library to set up OTA updates for an ESP32.
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

#include <OTAUpdate.h>

const char* ssid = "Replace with your ssid";
const char* password = "Replace with your password";

Updater ota(80); // create an Updater object with the Server's port equal to 80

void setup() {

    ota.WiFiInit(); // begin the WiFi
    ota.OTAInit();  // begin the Async Elegant OTA as well as the Server
    // Now open the Serial, you will see an IP adress, search IP-adress/update, upload the .bin file of your code project.

    // now you can put your setup code here, to run once:

}

void loop() {
    // put your main code here, to run repeatedly:

}
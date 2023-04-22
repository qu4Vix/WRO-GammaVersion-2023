# OTA Updater Library

**An Arduino library for setting up OTA Updates using Async Elegant OTA on ESP32.**

The OTAUpdater library provides an easy way to enable Over-The-Air (OTA) updates for your ESP32-based Arduino project using the AsyncElegantOTA library. OTA updates allow you to update your ESP32 firmware over your local Wi-Fi network, eliminating the need to physically connect to your device to update the firmware.

This library is part of GammaVersion's [repository](https://github.com/qu4Vix/WRO-GammaVersion-2023) for the [World Robot Olympiad](https://wro-association.org) 2023 season.

## AsyncElegantOTA

The AsyncElegantOTA library is developed by Ayush Sharma and licensed under the MIT License.
Copyright (c) 2018-2021 Ayush Sharma.

For more information, see the [AsyncElegantOTA GitHub repository](https://github.com/ayushsharma82/AsyncElegantOTA).

## Installation

### Arduino IDE

To use the OTAUpdater library in your Arduino project, follow these steps:

1. Download the latest release from the [releases page](https://github.com/qu4Vix/WRO-GammaVersion-2023/releases).
2. Extract the downloaded ZIP file to your Arduino libraries folder. On Windows, this is usually located in "Documents/Arduino/libraries". 
   - You can find the location of your sketchbook by going to **File > Preferences** in the Arduino IDE.
3. Include the OTAUpdater library in your Arduino sketch by adding #include \<OTAUpdater.h\> at the top of your sketch.

### Platform IO

To use the OTAUpdater library in your PlatformIO project, follow these steps:

1. Clone the OTAUpdater repository from GitHub to your local machine.
2. Copy the `OTAUpdater` folder from the cloned repository to your project's `lib` folder.
3. In your PlatformIO project, include the OTAUpdater library in your code by adding the following line at the beginning of your sketch:
```cpp
#include <OTAUpdater.h>
```

## Usage

### Library Usage

To use the OTAUpdater library in your ESP32-based Arduino project, follow these steps:

1. Create an instance of the OTAUpdater class: ´OTAUpdater updater´
2. In your ´setup()´ function, call the ´updater.begin()´ method to initialize the OTA update process, passing in the port of the member AsyncWebServer as parameter: ´updater.begin(port);´

That's it! Now you can update your ESP32 firmware over your local Wi-Fi network using the AsyncElegantOTA library.

### WebPage Usage

1. Open a web browser and navigate to the IP address of your ESP32 device. For example, if your ESP32 is connected to your local network and its IP address is 192.168.1.100, you would enter http://192.168.1.100/update into your web browser's address bar.
2. The ElegantOTA web page should now be displayed. Here you can upload a new firmware image to be flashed onto the ESP32 device.
3. Click the "Choose File" button to select the new firmware image from your computer's file system.
    - The firmware image is a binary file that contains the new software that you want to upload to the device. You can create a firmware image using a tool such as Arduino IDE or PlatformIO.
4. Once you have selected the firmware image, click the "Upload" button to start the update process. The progress bar will show the current status of the update.
5. After the update is complete, the device will automatically restart with the new firmware.

That's it! You have now updated the firmware on your ESP32 device using the ElegantOTA web page.

## API Reference

### `Updater`

The `Updater` class provides the following function:

`Updater(uint16_t port)`
Creates a new instance of the `Updater` class. It takes the port as argument.

`void WiFiInit()`
Initialices the WiFi connection for the ESP32 with the `ssid` and `password` definded.

`void OTAUpdate()`
Begins the Elegant OTA and the server.

## License

This library is released under the GNU Lesser General Public License, version 3.0. See the [LICENSE](LICENSE) file for more details.

## References

### LGPL-2.1 License
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32) - Official Arduino core for ESP32.
- [WiFI](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi) - ESP32 WiFi support library for Arduino. It is part of the core above.

### LGPL-3.0 License
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) - Asynchronous Web Server for ESP8266 and ESP32.
- [AsyncTCP](https://github.com/me-no-dev/AsyncTCP) - Asynchronous TCP Library for ESP32.

### MIT License
- [AsyncElegantOTA](https://github.com/ayushsharma82/AsyncElegantOTA) - ESP32 and ESP8266 Asynchronous Elegant Over The Air update library.

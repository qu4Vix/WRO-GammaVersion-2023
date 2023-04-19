# OTA Updater Library

**An Arduino library for setting up OTA Updates using Async Elegant OTA on ESP32.**

The OTAUpdater library provides an easy way to enable Over-The-Air (OTA) updates for your ESP32-based Arduino project using the AsyncElegantOTA library. OTA updates allow you to update your ESP32 firmware over your local Wi-Fi network, eliminating the need to physically connect to your device to update the firmware.

This library is part of GammaVersion's [repository](https://github.com/qu4Vix/WRO-GammaVersion-2023) for the [World Robot Olympiad](https://wro-association.org) 2023 season.

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

To use the OTAUpdater library in your ESP32-based Arduino project, follow these steps:

1. Create an instance of the OTAUpdater class: ´OTAUpdater updater´
2. In your ´setup()´ function, call the ´updater.begin()´ method to initialize the OTA update process, passing in your Wi-Fi network SSID and password as parameters: ´updater.begin("mySSID", "myPassword");´

That's it! Now you can update your ESP32 firmware over your local Wi-Fi network using the AsyncElegantOTA library.

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
# cubesat-spe

These are a pair of sketches used to evaluate the [SparkFun MicroMod Single Pair Ethernet Kit](https://www.sparkfun.com/products/19628) for use in CubeSats.

## Hardware
The hardware used in this project consists of the following:
- The [SparkFun MicroMod Single Pair Ethernet Kit](https://www.sparkfun.com/products/19628), containing
    - 2 [SparkFun MicroMod Single Pair Ethernet Function Board - ADIN1110](https://www.sparkfun.com/products/19038)
    - 2 [SparkFun MicroMod Main Board - Single](https://www.sparkfun.com/products/20748) Note: The kit we received contained the [latest version](https://www.sparkfun.com/products/20748) and the [previous version](https://www.sparkfun.com/products/18575) of the main boards. They were functionally identical for our purposes.
    - 1 [SparkFun MicroMod ESP32 Processor](https://www.sparkfun.com/products/16781)
    - 1 [SparkFun MicroMod Artemis Processor](https://www.sparkfun.com/products/16401)
    - 1 [Single Pair Ethernet Cable - 0.5m (Shielded)](https://www.sparkfun.com/products/19312)
- 1 [Adafruit LSM6DSOX 6 DoF Accelerometer and Gyroscope - STEMMA QT / Qwiic](https://www.adafruit.com/product/4438)
- 1 [Adafruit Triple-axis Magnetometer - LIS3MDL - STEMMA QT / Qwiic](https://www.adafruit.com/product/4479)
- 2 [STEMMA QT / Qwiic JST SH 4-Pin Cable - 50mm Long](https://www.adafruit.com/product/4399)

These sensors were chosen as the Adafruit STEMMA QT and SparkFun Qwiic connectors are compatible and the sensors are already in use in the [Ke Ao CubeSat project](https://www.teamlaniakea.com/keao).

## Setup
Following are directions on setting up the Arduino IDE for this project. This project has been tested on Arduino IDE v.1.8.19.

### Installing the board definitions
The following lines should be added to the Additional Boards Manager URLs list in the File > Preferences dialog:

`https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

After restarting Arduino IDE, the following boards should be available in the Tools > Board menu:
- ESP32 Arduino > SparkFun ESP32 MicroMod
- SparkFun Apollo3 > Artemis MicroMod Processor

Ensure the `esp32.ino` sketch is uploaded to the ESP32 microcontroller using the SparkFun ESP32 MicroMod board definition, and similarly ensure the `artemis.ino` sketch is uploaded to the Artemis microcontroller using the Artemis MicroMod Processor board definition.

### Installing the Arduino libaries
The following Arduino libaries should be installed:

- SparkFun ADIN1110 Arduino Library
- Adafruit LSM6DS
- Adafruit LIS3MDL
- Adafruit BusIO
- Adafruit Unified Sensor

These libaries can be found verbatim in the Library Manager, found under Tools > Manage Libraries...



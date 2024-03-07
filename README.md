# Arduino (C++) Bluetooth Controller for Mecanum Battle Robot
## Background
This repository contains the mecanum control system implemented in Arduino C++.
It uses Bluetooth Low Energy to receive data over Bluetooth characteristics. Uses default Arduino BLE library and an Euler angle library provided below.
Controls Roboclaws and has IMU integration to guide field-oriented control.

## Prerequisits
# Arduino IDE
Download Arduino IDE (we use the legacy IDE v1.8.19 found at https://www.arduino.cc/en/software)
Download .ino file and place in directory

# ESP32
Install board ESP32-Dev-Module from https://github.com/espressif/arduino-esp32

ESP32 Datasheet found here: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-da_datasheet_en.pdf)

# IMU Libraries (need all 3)
Adafruit BNO055: https://github.com/adafruit/Adafruit_BNO055
Adafruit sensor: https://github.com/adafruit/Adafruit_Sensor
Adafruit Bus IO: https://github.com/adafruit/Adafruit_BusIO

# Roboclaw libraries:
https://resources.basicmicro.com/using-the-roboclaw-arduino-library/

## Field Oriented Control
This code reads an IMU and uses an Euler angle library to find the direction the bot is currently facing (yaw). The operator using joysticks simply gives the bot a direction
and the bot will automatically correct using it's known orientation angle such that the bot drives in a direction that is from the operator's standpoint,
assuming the operator stands in the same spot relative to the bot.

## Libraries (Find in Arduino IDE and replace IMU with desired device)
```
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <RoboClaw.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <esp_task_wdt.h>
```

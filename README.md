# Arduino (C++) Bluetooth Controller for Mecanum Battle Robot
## Background
This repository contains the mecanum control system implemented in Arduino C++.
It uses Bluetooth Low Energy to receive data over Bluetooth characteristics. Uses default Arduino BLE library and an Euler angle library provided below.
Controls Roboclaws and has IMU integration to guide field-oriented control.

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
#include "ICM_20948.h"
```

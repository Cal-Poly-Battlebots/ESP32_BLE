/*
   ESP32 Bluetooth Low Energy Implementation
   BattleBots Capstone -- Fall 2023
   Author(s): Aaron Rosen (CPE) & ChatGPT (AI)

   Sets up a BLE server and reads 3 characteristics
   Uses FreeRTOS underneath Arduino execution
   For communication with an iOS app
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <RoboClaw.h>
#include "soc/dport_reg.h"
#include "soc/gpio_reg.h"



//---------RTOS--------------------
// define two tasks for reading from ble and writing to serial
void TaskWriteToSerial(void *pvParameters);
void TaskReadFromBLE(void *pvParameters);

// Define Queue handle
QueueHandle_t QueueHandle;
const int QueueElementSize = 50;

typedef struct {
  float angle;
  float magnitude;
} input_t;

//---------RoboClaw----------------
RoboClaw roboclaw(&Serial2, 10000);
#define address_right 0x80 // 128
#define address_left 0x82 // 130

#define PI 3.14159

//---------BLE---------------------
BLECharacteristic *joystickCharacteristic;
BLECharacteristic *swipeCharacteristic;
BLECharacteristic *abilityCharacteristic;
bool deviceConnected = false;

float joystickAngle = 0; // Degrees
float joystickMagnitude = 0;
float swipeAngle = 0; // Degrees
float swipeMagnitude = 0;
String abilityBar = "";

float maxMagnitude = 50.0;
int maxMotorPower = 127;

// UART service UUID data
#define SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define JOYSTICK_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define SWIPE_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define ABILITY_UUID     "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      Serial.println("Device connected.");
      deviceConnected = true;
    };
    void onDisconnect(BLEServer *pServer) {
      Serial.println("Device disconnected.");
      deviceConnected = false;
      pServer->getAdvertising()->start(); // Restart advertising
    }
};

class MyReadCallbacks : public BLECharacteristicCallbacks {
    input_t command;
    void onWrite(BLECharacteristic *pCharacteristic) {
      // Serial.println("Entered the asynch read");
      if (QueueHandle != NULL && uxQueueSpacesAvailable(QueueHandle) > 0)
      {
        if (pCharacteristic == joystickCharacteristic)
        {
          // Read data from joystickCharacteristic
          std::string value = pCharacteristic->getValue();

          // Parse received data (assuming it's in format "angle,magnitude")
          int commaIndex = value.find(',');
          if (commaIndex != -1 && value.length() > commaIndex + 1)
          {
            String angleStr = value.substr(0, commaIndex).c_str();
            String magnitudeStr = value.substr(commaIndex + 1).c_str();

            // Update ESP32 variables with parsed values
            command.angle = angleStr.toFloat();
            command.magnitude = magnitudeStr.toFloat();
          }
        }

        int ret = xQueueSend(QueueHandle, (void*) &command, 0);

        if (ret == pdTRUE) {

        }
        else if (ret == errQUEUE_FULL) {
          Serial.println("Queue full error");
        }

      }

      // else if (pCharacteristic == swipeCharacteristic) {
      //   // Read data from joystickCharacteristic
      //   std::string value = pCharacteristic->getValue();

      //   // Parse received data (assuming it's in format "angle,magnitude")
      //   int commaIndex = value.find(',');
      //   if (commaIndex != -1 && value.length() > commaIndex + 1) {
      //     String angleStr = value.substr(0, commaIndex).c_str();
      //     String magnitudeStr = value.substr(commaIndex + 1).c_str();

      //     // Update ESP32 variables with parsed values
      //     swipeAngle = angleStr.toFloat();
      //     swipeMagnitude = magnitudeStr.toFloat();
      //   }
      // } else if (pCharacteristic == abilityCharacteristic) {
      //   std::string value = pCharacteristic->getValue();
      //   abilityBar = String(value.c_str());
      // }
    }
};

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  };// Add a delay to prevent issues initializing BLE after flashing
  BLEDevice::init("ESP32_BLE_Test"); // Set BLE device name
  BLEServer *pServer = BLEDevice::createServer(); // Create BLE server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the queue handle with <QueueElementSize> number of elements
  QueueHandle = xQueueCreate(QueueElementSize, sizeof(input_t));

  // Check if the queue was created
  if (QueueHandle == NULL) {
    Serial.println("Queue could not be created. Program is halted.");
    while (1) delay(1000);
  }

  // Write to Serial task for debugging
  xTaskCreate(
    TaskWriteToSerial,
    "Task Write To Serial", // name for convenience
    2048,                   // stack size
    NULL,                   // no params
    3,                      // prio of 2
    NULL                    // task handle is not used here
  );

  // xTaskCreate(
  //   TaskReadFromBLE,
  //     "Task Read From BLE",
  //     2048,
  //     NULL,
  //     3,
  //     NULL
  // );

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristics
  joystickCharacteristic = pService->createCharacteristic(
                             JOYSTICK_UUID,
                             BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  joystickCharacteristic->addDescriptor(new BLE2902());
  joystickCharacteristic->setCallbacks(new MyReadCallbacks());

  // swipeCharacteristic = pService->createCharacteristic(
  //                         SWIPE_UUID,
  //                         BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  // swipeCharacteristic->addDescriptor(new BLE2902());
  // swipeCharacteristic->setCallbacks(new MyReadCallbacks());

  // abilityCharacteristic = pService->createCharacteristic(
  //                           ABILITY_UUID,
  //                           BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  // abilityCharacteristic->addDescriptor(new BLE2902());
  // abilityCharacteristic->setCallbacks(new MyReadCallbacks());

  pService->start(); // Start the service
  pServer->getAdvertising()->start(); // Start advertising
  Serial.println("Waiting for a client connection...");

  // Initialize RoboClaws with baud rate
  roboclaw.begin(38400);// Should we go to 460800 bps
}

void loop() {
  // Mecanum drive based on the BLE readings
  // float power = joystickMagnitude / maxMagnitude;// Normalize power 0-1
  // float anglesToRadians = 0.01745329252;
  // float sine = sin(((joystickAngle * anglesToRadians) - (PI / 4)));
  // float cosine = cos(((joystickAngle * anglesToRadians) - (PI / 4)));
  // float maximum = max(abs(sine), abs(cosine));// 0 to 1

  // // sine yields -1.0 to 1.0, swipe magnitude is 0-50, then -50.0 to 50.0 is normalized to -1.0 to 1.0
  // float turn = sin((swipeAngle * anglesToRadians)) * swipeMagnitude / maxMagnitude;
  // int turnNormalized = turn * maxMotorPower;// -maxMotorPower to maxMotorPower

  // // Motor speeds 0 to max motor power
  // // Negative sign because motors are on the outside
  // int leftFront = power * maxMotorPower * sine / maximum + turnNormalized;// turn is 0-maxMotorPower added onto translation power 0-maxMotorPower
  // int rightFront = -(power * maxMotorPower * cosine / maximum - turnNormalized);
  // int leftRear = power * maxMotorPower * cosine / maximum + turnNormalized;
  // int rightRear = -(power * maxMotorPower * sine / maximum - turnNormalized);

  // // If one is overpowered, make sure it maxes out and the rest are scaled down with it
  // if (power * maxMotorPower + abs(turnNormalized) > maxMotorPower) {
  //   // -maxMotorPower to maxMotorPower
  //   leftFront = (leftFront / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
  //   rightFront = (rightFront / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
  //   leftRear = (leftRear / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
  //   rightRear = (rightRear / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
  // }

  // // -127 to 127 --convert--> 0 to 127
  // leftRear = (leftRear + 127) / 2;
  // leftFront = (leftFront + 127) / 2;
  // rightRear = (rightRear + 127) / 2;
  // rightFront = (rightFront + 127) / 2;

  // // Control motors
  // roboclaw.ForwardBackwardM2(address_left, leftRear);
  // roboclaw.ForwardBackwardM1(address_right, leftFront);
  // roboclaw.ForwardBackwardM1(address_left, rightRear);
  // roboclaw.ForwardBackwardM2(address_right, rightFront);
    // DEBUG
  delay(1000);
}

void TaskMoveRobot(void *pvParamters) {
  input_t command;
  for (;;) { // A task doesn't return or exit
    //

    if (QueueHandle != NULL) {
      int ret = xQueueReceive(QueueHandle, &command, portMAX_DELAY);
      if (ret == pdPASS) {

      }
    }
  }
}

void TaskWriteToSerial(void *pvParameters) { // This is a task.
  input_t command;
  for (;;) { // A Task shall never return or exit.
    // One approach would be to poll the function (uxQueueMessagesWaiting(QueueHandle) and call delay if nothing is waiting.
    // The other approach is to use infinite time to wait defined by constant `portMAX_DELAY`:
    if (QueueHandle != NULL) { // Sanity check just to make sure the queue actually exists
      int ret = xQueueReceive(QueueHandle, &command, portMAX_DELAY);
      if (ret == pdPASS) {
        // if the angle and mag are not both 0
        if(command.angle != 0 && command.magnitude != 0) {
        // The message was successfully received - send it back to Serial port and "Echo: "
          Serial.printf("Input angle: %f, Input Mag: %f\n", command.angle, command.magnitude);
        // The item is queued by copy, not by reference, so lets free the buffer after use.
         }
      } else if (ret == pdFALSE) {
        Serial.println("The `TaskWriteToSerial` was unable to receive data from the Queue");
      }
    } // Sanity check
  } // Infinite loop
}

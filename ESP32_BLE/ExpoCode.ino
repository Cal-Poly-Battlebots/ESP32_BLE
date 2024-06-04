/*
   ESP32 Bluetooth Low Energy Implementation
   BattleBots Capstone -- Fall 2023
   Author(s): Aaron Rosen (CPE) & ChatGPT (AI)

   Sets up a BLE server and reads 3 characteristics
   Uses FreeRTOS underneath Arduino execution
   For communication with an iOS app

   Mecanum Field-Oriented Control implemented using an IMU
   Converts sensor data to Euler Angles
*/
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

//3 seconds WDT
#define WDT_TIMEOUT 3
#define KILL_PIN 32

#define LED_PIN 25

//---------RoboClaw----------------
RoboClaw roboclaw(&Serial2, 10000);
#define address_right 0x80 // 128
#define address_left 0x82 // 130

#define PI 3.14159

#define TURN_RATIO 0.55
#define POWER_ADJ 0.80

//---------IMU---------------------
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define IMU_ADDR 0x28

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_ADDR);

bool fieldOriented = true;
bool weaponEnable = false;
bool estop = true;

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
// maximum value in code is 127 for the maxMotorPower
int maxMotorPower = 32;

//---------Weapon Motor---------------------
// setting PWM properties for weapon motor
const int weaponPin = 3;  // 16 corresponds to GPIO16
const int freq = 200; // can range from 50 to 200 (200 has best resolution for speed control)
const int weaponChannel = 0; // timer channel
const int resolution = 9; // used to give min max values for the ledcWrite for the weapon motor 
const int weaponOn = 160;
const int weaponOff = 154;
const int deadZone = 2;
// max value for ledcwrite is undedermined at the moment since we are essentially sending servo control and duty cycle for full speed is not actually 100% duty


// UART service UUID data
#define SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define JOYSTICK_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define SWIPE_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define ABILITY_UUID     "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

/*
   Kills motors and delays for ~1s
*/
void killMotors() {
  // Disconnect power from the motors
  digitalWrite(KILL_PIN, HIGH);
  ledcWrite(weaponChannel, weaponOff); // 154 to turn weapon off, 157 to turn weapon on

  // Turn off motors
  roboclaw.ForwardM2(address_left, 0);
  roboclaw.ForwardM1(address_right, 0);
  roboclaw.ForwardM1(address_left, 0);
  roboclaw.ForwardM2(address_right, 0);

  delay(1000);
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Device connected.");

      deviceConnected = true;
    };
    void onDisconnect(BLEServer *pServer) {
      digitalWrite(LED_PIN, LOW);
      killMotors();
      Serial.println("Device disconnected.");
      deviceConnected = false;
      pServer->getAdvertising()->start(); // Restart advertising
      killMotors();
    }
};

class MyReadCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      if (pCharacteristic == joystickCharacteristic) {
        // Read data from joystickCharacteristic
        std::string value = pCharacteristic->getValue();

        // Parse received data (assuming it's in format "angle,magnitude")
        int commaIndex = value.find(',');
        if (commaIndex != -1 && value.length() > commaIndex + 1) {
          String angleStr = value.substr(0, commaIndex).c_str();
          String magnitudeStr = value.substr(commaIndex + 1).c_str();

          // Update ESP32 variables with parsed values
          joystickAngle = angleStr.toFloat();
          joystickMagnitude = magnitudeStr.toFloat();
          if (joystickMagnitude < deadZone && joystickMagnitude > -deadZone) {
            joystickMagnitude = 0;
          }
        }
      } else if (pCharacteristic == swipeCharacteristic) {
        // Read data from joystickCharacteristic
        std::string value = pCharacteristic->getValue();

        // Parse received data (assuming it's in format "angle,magnitude")
        int commaIndex = value.find(',');
        if (commaIndex != -1 && value.length() > commaIndex + 1) {
          String angleStr = value.substr(0, commaIndex).c_str();
          String magnitudeStr = value.substr(commaIndex + 1).c_str();

          // Update ESP32 variables with parsed values
          swipeAngle = angleStr.toFloat();
          swipeMagnitude = magnitudeStr.toFloat();
          if (swipeMagnitude < deadZone && swipeMagnitude > -deadZone) {
            swipeMagnitude = 0;
          }
        }
      } else if (pCharacteristic == abilityCharacteristic) {
        std::string value = pCharacteristic->getValue();
        // Process the ability string
        if (value.length() > 2) {
          Serial.print("String: ");
          Serial.println(String(value.c_str()));

          // "100" means weapon enable
          weaponEnable = (value[0] == '1');
          if (weaponEnable && !estop) {
            Serial.println("Weapon on");
            ledcWrite(weaponChannel, weaponOn); // 154 to turn weapon off, 157 to turn weapon on
          } else {
            Serial.println("Weapon off");
            ledcWrite(weaponChannel, weaponOff); // 154 to turn weapon off, 157 to turn weapon on
          }

          // "010" means field oriented enable
          fieldOriented = (value[1] == '1');
          if (fieldOriented) {
            Serial.println("FO on");
          } else {
            Serial.println("FO off");
          }

          // Toggle Kill Pin ESTOP using "000"
          estop = (value[2] == '0');
          if (estop) {
            // ESTOP on
            Serial.println("killing");
            digitalWrite(KILL_PIN, HIGH);
            ledcWrite(weaponChannel, weaponOff); // 154 to turn weapon off, 157 to turn weapon on
          } else {
            // ESTOP off
            digitalWrite(KILL_PIN, LOW);
          }
        }
      }
    }
};

float readIMU() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The processing sketch expects data as roll, pitch, heading */
  //  Serial.print(F("Orientation: "));
  //  Serial.print((float)event.orientation.x);
  //  Serial.print(F(" "));
  //  Serial.print((float)event.orientation.y);
  //  Serial.print(F(" "));
  //  Serial.print((float)event.orientation.z);
  //  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  //  Serial.print(F("Calibration: "));
  //  Serial.print(sys, DEC);
  //  Serial.print(F(" "));
  //  Serial.print(gyro, DEC);
  //  Serial.print(F(" "));
  //  Serial.print(accel, DEC);
  //  Serial.print(F(" "));
  //  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
  //
  //  Serial.print("Facing Angle: ");
  //  Serial.println(event.orientation.x);

  return event.orientation.x;
}

void setup() {
  Serial.begin(9600);

  // ------------ Initialize WDT -----------
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  pinMode(KILL_PIN, OUTPUT);
  killMotors();
  // weapon motor setup on next two
  ledcSetup(weaponChannel, freq, resolution);
  ledcAttachPin(weaponPin, weaponChannel);

  esp_task_wdt_reset();

  // Debug LED off
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize RoboClaws with baud rate
  roboclaw.begin(38400);
  delay(400);

  esp_task_wdt_reset();

  // Immediately set roboclaw to 0 in case of reset
  killMotors();

  delay(1000); // Add a delay to prevent issues initializing BLE after flashing

  esp_task_wdt_reset();

  BLEDevice::init("ESP32_BLE"); // Set BLE device name
  BLEServer *pServer = BLEDevice::createServer(); // Create BLE server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristics
  joystickCharacteristic = pService->createCharacteristic(
                             JOYSTICK_UUID,
                             BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  joystickCharacteristic->addDescriptor(new BLE2902());
  joystickCharacteristic->setCallbacks(new MyReadCallbacks());

  swipeCharacteristic = pService->createCharacteristic(
                          SWIPE_UUID,
                          BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  swipeCharacteristic->addDescriptor(new BLE2902());
  swipeCharacteristic->setCallbacks(new MyReadCallbacks());

  abilityCharacteristic = pService->createCharacteristic(
                            ABILITY_UUID,
                            BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  abilityCharacteristic->addDescriptor(new BLE2902());
  abilityCharacteristic->setCallbacks(new MyReadCallbacks());


  esp_task_wdt_reset();

  pService->start(); // Start the service
  pServer->getAdvertising()->start(); // Start advertising
  Serial.println("Waiting for a client connection...");


  esp_task_wdt_reset();

  // ------------ Initialize IMU -----------
  /* Initialize the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");

    esp_task_wdt_reset();
    while (1);
  }

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void loop() {

  if (!deviceConnected) {
    killMotors();
    digitalWrite(LED_PIN, LOW);
    Serial.println("Waiting for a client connection...");

    // Check in with watchdog
    esp_task_wdt_reset();
    return;
  }

  // Read IMU first get angle 0-360
  float angleFacing = readIMU();

  if (!fieldOriented) {
    angleFacing = 0;
  }

  if (!estop) {
    // Mecanum drive based on the BLE readings
    float power = joystickMagnitude / maxMagnitude * POWER_ADJ;// Normalize power 0-1
    float anglesToRadians = 0.01745329252;
    // 360 values of precision
    int trueAngle = (int) round((joystickAngle - angleFacing)) % 360;
    float finalAngle = ((trueAngle * anglesToRadians) - (PI / 4));

    float sine = sin(finalAngle);
    float cosine = cos(finalAngle);
    float maximum = max(abs(sine), abs(cosine));// 0 to 1

    // sine yields -1.0 to 1.0, swipe magnitude is 0-50, then -50.0 to 50.0 is normalized to -1.0 to 1.0
    float turn = sin((swipeAngle * anglesToRadians)) * swipeMagnitude / maxMagnitude;
    int turnNormalized = turn * maxMotorPower * TURN_RATIO;// -maxMotorPower to maxMotorPower, scaled by defined amount

    // Motor speeds 0 to max motor power
    // Negative sign because motors are on the outside
    int leftFront = power * maxMotorPower * sine / maximum + turnNormalized;// turn is 0-maxMotorPower added onto translation power 0-maxMotorPower
    int rightFront = -(power * maxMotorPower * cosine / maximum - turnNormalized);
    int leftRear = power * maxMotorPower * cosine / maximum + turnNormalized;
    int rightRear = -(power * maxMotorPower * sine / maximum - turnNormalized);

    // If one is overpowered, make sure it maxes out and the rest are scaled down with it
    if (power * maxMotorPower + abs(turnNormalized) > maxMotorPower) {
      // -maxMotorPower to maxMotorPower
      leftFront = (leftFront / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
      rightFront = (rightFront / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
      leftRear = (leftRear / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
      rightRear = (rightRear / (power * maxMotorPower + abs(turnNormalized))) * maxMotorPower;
    }

    // -127 to 127 --convert--> 0 to 127
    leftRear = (leftRear + 127) / 2;
    leftFront = (leftFront + 127) / 2;
    rightRear = (rightRear + 127) / 2;
    rightFront = (rightFront + 127) / 2;

    // Control motors
    roboclaw.ForwardBackwardM2(address_left, leftRear);
    roboclaw.ForwardBackwardM1(address_right, leftFront);
    roboclaw.ForwardBackwardM1(address_left, rightRear);
    roboclaw.ForwardBackwardM2(address_right, rightFront);
  }
  else {
    killMotors();
  }

  

  // Check in with Watchdog
  esp_task_wdt_reset();
}
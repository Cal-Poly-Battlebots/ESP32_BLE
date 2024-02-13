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
#include "ICM_20948.h"

//---------RoboClaw----------------
RoboClaw roboclaw(&Serial2, 10000);
#define address_right 0x80 // 128
#define address_left 0x82 // 130

#define PI 3.14159

#define TURN_RATIO 0.55

//---------IMU---------------------
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;

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
      joystickMagnitude = 0;
      swipeMagnitude = 0;
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
        }
      } else if (pCharacteristic == abilityCharacteristic) {
        std::string value = pCharacteristic->getValue();
        abilityBar = String(value.c_str());
      }
    }
};

float readIMU() {
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      Serial.print("Yaw: ");
      Serial.println(yaw);
      return yaw;
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  return 0;
}

void setup() {
  Serial.begin(9600);
  delay(1000); // Add a delay to prevent issues initializing BLE after flashing
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

  pService->start(); // Start the service
  pServer->getAdvertising()->start(); // Start advertising
  Serial.println("Waiting for a client connection...");

  // Initialize RoboClaws with baud rate
  roboclaw.begin(38400);// Should we go to 460800 bps

  // Initialize IMU
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  bool initialized = false;
  while (!initialized) {
    // Try calling begin()
    myICM.begin(WIRE_PORT, AD0_VAL);

    // Check status and reconnect if needed
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying to connect again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  
  Serial.println(myICM.statusString());
  
  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
    Serial.println("DMP enabled!");
  } else {
    Serial.println("Enable DMP failed!");
    Serial.println("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    while (1)
      ; // Do nothing more
  }
  
}

void loop() {
  // Read IMU first
  float angleFacing = readIMU();
  // Adjust -180 to 180 ---> 0 to 360
  if (angleFacing < 0) {
    angleFacing += 360;
  }
  
  // Mecanum drive based on the BLE readings
  float power = joystickMagnitude / maxMagnitude;// Normalize power 0-1
  float anglesToRadians = 0.01745329252;
  float controllerAngle = ((joystickAngle * anglesToRadians) - (PI / 4));
  // 360 values of precision
  int finalAngle = (int) round((controllerAngle - angleFacing)) % 360;
  float sine = sin(finalAngle);
  float cosine = cos(finalAngle);
  float maximum = max(abs(sine), abs(cosine));// 0 to 1

  // sine yields -1.0 to 1.0, swipe magnitude is 0-50, then -50.0 to 50.0 is normalized to -1.0 to 1.0
  float turn = sin((swipeAngle * anglesToRadians)) * swipeMagnitude / maxMagnitude;
  int turnNormalized = turn * maxMotorPower * TURN_RATIO;// -maxMotorPower to maxMotorPower

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

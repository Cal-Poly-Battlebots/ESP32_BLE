#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLECharacteristic *joystickCharacteristic;
BLECharacteristic *swipeCharacteristic;
BLECharacteristic *abilityCharacteristic;
bool deviceConnected = false;

float joystickAngle = 0;
float joystickMagnitude = 0;
float swipeAngle = 0;
float swipeMagnitude = 0;
String abilityBar = "";

// UART service UUID data
#define SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define JOYSTICK_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define SWIPE_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define ABILITY_UUID     "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyReadCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
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

void setup() {
  Serial.begin(9600);
  BLEDevice::init("ESP32_BLE"); // Set BLE device name
  BLEServer *pServer = BLEDevice::createServer(); // Create BLE server
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristics
  joystickCharacteristic = pService->createCharacteristic(
      JOYSTICK_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  joystickCharacteristic->addDescriptor(new BLE2902());
  joystickCharacteristic->setCallbacks(new MyReadCallbacks());

  swipeCharacteristic = pService->createCharacteristic(
      SWIPE_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  swipeCharacteristic->addDescriptor(new BLE2902());
  swipeCharacteristic->setCallbacks(new MyReadCallbacks());

  abilityCharacteristic = pService->createCharacteristic(
      ABILITY_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  abilityCharacteristic->addDescriptor(new BLE2902());
  abilityCharacteristic->setCallbacks(new MyReadCallbacks());

  pService->start(); // Start the service
  pServer->getAdvertising()->start(); // Start advertising
  Serial.println("Waiting for a client connection...");
}

void loop() {
  // Handle other tasks, if any, within loop
  // No continuous BLE operations are necessary in loop
  // Values will be read when the characteristics are accessed by the iOS app
  Serial.println("Joystick (Angle, Magnitude): " + String(joystickAngle) + ", " + String(joystickMagnitude));
  Serial.println("Swipe (Angle, Magnitude): " + String(swipeAngle) + ", " + String(swipeMagnitude));
  Serial.println("Ability Bar (Binary String): " + abilityBar);
  delay(50); // Add a delay to avoid rapid Serial prints
}

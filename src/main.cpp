#include <Arduino.h>
#include "ArduinoJson.h"

#include <string>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define RXD2 16
#define TXD2 17

#define I2C_SDA 21
#define I2C_SCL 22

#define ULTRASONIC_DETECTOR__COUNT 4

const int TRIG_PINS[ULTRASONIC_DETECTOR__COUNT] = {12, 27, 25, 32};
const int ECHO_PINS[ULTRASONIC_DETECTOR__COUNT] = {13, 14, 26, 33};



class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic);
};

class ServerCallbacks:  public BLEServerCallbacks {
  void onConnect(BLEServer* pServer);
  void onDisconnect(BLEServer* pServer);
};



class Lidar {
  SparkFun_VL53L5CX sensor;
  VL53L5CX_ResultsData measurement;
  
  int imageResolution = 0;
  int imageWidth = 0;
  bool lidarAvailable = false;
  bool isEnabled = false;
  int measurements[64] = {0};

  void setupI2C();
  void setupResolution();
  void setupRangingMode();
  void setPowerModeSleep();
  void setPowerModeWakeup();
  void setupIntegrationTime();
  void setupSharpener();
  void setupTargetOrder();
  void setupRanging();

  public:
    Lidar() {};
    void setup();
    void disable();
    void startMeasurement();
    void stopMeasurement();
    void measure();
    int* getMeasurements();
    bool getIsEnabled();
};

void Lidar::setupI2C() {
  Serial.println("Setting up I2C...");
  Wire.begin();
  Wire.setClock(1000000);
  Serial.println("I2C setup finished.");
}

void Lidar::setupResolution() {
  Serial.println("Setting up resolution...");

  sensor.setResolution(VL53L5CX_RESOLUTION_8X8);
  imageResolution = sensor.getResolution();
  imageWidth = sqrt(imageResolution);

  if (sensor.isConnected()) {
    Serial.println("VL53L5CX is connected");
  } else {
    Serial.println("VL53L5CX is not connected");
    lidarAvailable = false;
  }

  Serial.println("Resolution setup finished.");
}

void Lidar::setupRangingMode() {
  Serial.println("Setting up ranging mode...");
  if (sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS)) {
    Serial.println("VL53L5CX set to autonomous mode");
  } else {
    Serial.println(F("Error setting ranging mode."));
    lidarAvailable = false;
  }
}

void Lidar::setPowerModeSleep() {
  Serial.println("Setting up power mode (sleep)...");
  if  (sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP)) {
    Serial.println("VL53L5CX set to sleep mode");
  } else {
    Serial.println("VL53L5CX failed to set to sleep mode");
    lidarAvailable = false;
  }
}

void Lidar::setPowerModeWakeup() {
  Serial.println("Setting up power mode (wakeup)...");
  if  (sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
    Serial.println("VL53L5CX set to wakeup mode");
  } else {
    Serial.println("VL53L5CX failed to set to wakeup mode");
    lidarAvailable = false;
  }
}

void Lidar::setupIntegrationTime() {
  Serial.println("Setting up integration time...");
  if (sensor.setIntegrationTime(100)) {
    Serial.println("VL53L5CX integration time set to 100ms");
  } else {
    Serial.println("VL53L5CX failed to set integration time");
    lidarAvailable = false;
  }
}

void Lidar::setupSharpener() {
  Serial.println("Setting up sharpener...");
  if (sensor.setSharpenerPercent(19)) {
    Serial.println("VL53L5CX sharpener set to 19 percent");
  } else {
    Serial.println("VL53L5CX failed to set sharpener");
    lidarAvailable = false;
  }
}

void Lidar::setupTargetOrder() {
  Serial.println("Setting up target order...");
  if (sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST)) {
    Serial.println("VL53L5CX target order set to closest");
  } else {
    Serial.println("VL53L5CX failed to set target order");
    lidarAvailable = false;
  }
}

void Lidar::setup() {
  setupI2C();

  lidarAvailable = sensor.begin();

  if (lidarAvailable) setupResolution();
  if (lidarAvailable) setupRangingMode();
  if (lidarAvailable) setupIntegrationTime();
  if (lidarAvailable) setupSharpener();
  if (lidarAvailable) setupTargetOrder();

  isEnabled = true;
}

void Lidar::disable() {
  isEnabled = false;
  stopMeasurement();
}

void Lidar::startMeasurement() {
  Serial.println("Starting Lidar measurement...");
  
  setPowerModeWakeup();
  
  if (sensor.startRanging()) {
    Serial.println("VL53L5CX ranging started");
  } else {
    Serial.println("VL53L5CX failed to start ranging");
    lidarAvailable = false;
  }

}

void Lidar::stopMeasurement() {
  Serial.println("Stopping Lidar measurement...");

  setPowerModeSleep();

  if (sensor.stopRanging()) {
    Serial.println("VL53L5CX ranging stopped");
  } else {
    Serial.println("VL53L5CX failed to stop ranging");
    lidarAvailable = false;
  }
}

void Lidar::measure() {
  int idx = 0;
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurement)) {
      for (int y = 0; y < imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          measurements[idx] = measurement.distance_mm[y + x];
          idx++;
        }
      }
    }
  }
}

bool Lidar::getIsEnabled() {
  return isEnabled;
}

int* Lidar::getMeasurements() {
  return measurements;
}



enum LunaMessageState {
  HEADER1 = 0x01,
  HEADER2 = 0x02,
  DISTANCE_LOW = 0x03,
  DISTANCE_HIGH = 0x04,
  STRENGTH_LOW = 0x05,
  STRENGTH_HIGH = 0x06,
  TEMPERATURE_LOW = 0x07,
  TEMPERATURE_HIGH = 0x08,
  CHECKSUM = 0x09
};

class Luna {
  bool isEnabled = false; 
  
  int distance;
  int signalStrength;
  float temperature;
  unsigned char checksum;

  unsigned char uartData[9];
  const int HEADER = 0x59;

  LunaMessageState messageState = LunaMessageState::HEADER1;

  void readUartMessage();
  void interpretMeasurementData();
  void printMeasurementData();
  
  public:
    Luna() {};
    void setup();
    void disable();
    void measure();    
    int getDistance();
    bool getIsEnabled(); 
};

void Luna::setup() {
  Serial.println("Setting up Luna LiDAR...");
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  isEnabled = true;
  Serial.println("Luna setup finished.");
}

void Luna::disable() {
  isEnabled = false;
}

bool Luna::getIsEnabled() {
  return isEnabled;
}

void Luna::readUartMessage() {
  while (messageState != LunaMessageState::CHECKSUM) {
      if (Serial2.available()) {
          switch (messageState) {
              case LunaMessageState::HEADER1:
                  uartData[0] = Serial2.read();
                  if (uartData[0] == 0x59) {
                      checksum = uartData[0];
                      messageState = LunaMessageState::HEADER2;
                  }
                  break;
              case LunaMessageState::HEADER2:
                  uartData[1] = Serial2.read();
                  if (uartData[1] == 0x59) {
                      checksum += uartData[1];
                      messageState = LunaMessageState::DISTANCE_LOW;
                  } else {
                      messageState = LunaMessageState::HEADER1;
                  }
                  break;
              case LunaMessageState::DISTANCE_LOW:
                  uartData[2] = Serial2.read();
                  checksum += uartData[2];
                  messageState = LunaMessageState::DISTANCE_HIGH;
                  break;
              case LunaMessageState::DISTANCE_HIGH:
                  uartData[3] = Serial2.read();
                  checksum += uartData[3];
                  messageState = LunaMessageState::STRENGTH_LOW;
                  break;
              case LunaMessageState::STRENGTH_LOW:
                  uartData[4] = Serial2.read();
                  checksum += uartData[4];
                  messageState = LunaMessageState::STRENGTH_HIGH;
                  break;
              case LunaMessageState::STRENGTH_HIGH:
                  uartData[5] = Serial2.read();
                  checksum += uartData[5];
                  messageState = LunaMessageState::TEMPERATURE_LOW;
                  break;
              case LunaMessageState::TEMPERATURE_LOW:
                  uartData[6] = Serial2.read();
                  checksum += uartData[6];
                  messageState = LunaMessageState::TEMPERATURE_HIGH;
                  break;
              case LunaMessageState::TEMPERATURE_HIGH:
                  uartData[7] = Serial2.read();
                  checksum += uartData[7];
                  messageState = LunaMessageState::CHECKSUM;
                  break;
          }
      }
  }

  uartData[8] = Serial2.read();
}

void Luna::interpretMeasurementData() {
  distance = uartData[2] + uartData[3]*256;
  signalStrength = uartData[4] + uartData[5]*256;
  temperature = uartData[6] + uartData[7] *256;
  temperature = temperature/8 - 256;                              

  while(Serial2.available()) {
      Serial2.read();
  }

  delay(100);
}

void Luna::printMeasurementData() {
  Serial.print("dist = ");
  Serial.print(distance);
  Serial.print('\n');
  Serial.print("strength = ");
  Serial.print(signalStrength);
  Serial.print('\n');
  Serial.print("\t Chip Temprature = ");
  Serial.print(temperature);
  Serial.println(" celcius degree");
}

int Luna::getDistance() {
  return distance;
}

void Luna::measure() {
  readUartMessage();

  if (uartData[8] == checksum) {
      interpretMeasurementData();
  }

  messageState = LunaMessageState::HEADER1;
}



class UltrasonicDetector {
  int echoPin;
  int trigPin;
  int id;
  int distance;

  public:
    UltrasonicDetector(int echoPin, int trigPin, int id);
    void setup();
    void measure();
    void printResult();
    int getDistance();
};

UltrasonicDetector::UltrasonicDetector(int echoPin, int trigPin, int id) {
  this->echoPin = echoPin;
  this->trigPin = trigPin;
  this->id = id;
}

void UltrasonicDetector::setup() {
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

void UltrasonicDetector::measure() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH) / 58.2;
}

void UltrasonicDetector::printResult() {
  Serial.print("[Ultrasonic]");
  Serial.print("[");
  Serial.print(echoPin);
  Serial.print("]");
  Serial.print("[");
  Serial.print(trigPin);
  Serial.print("]");
  Serial.print(": ");
  Serial.println(distance);
}

int UltrasonicDetector::getDistance() {
  return distance;
}



class UltrasonicDetectors {
  UltrasonicDetector* detectors[4];
  int count = 0;
  bool isEnabled = false;

  public:
    UltrasonicDetectors() {};
    void setup();
    void disable();
    void addDetector(int echoPin, int trigPin, int id);
    void clearDetectors();
    void measure();
    void printResults();
    int getCount();
    int* getDistances();
    int getDistance(int idx);
    bool getIsEnabled();
};

void UltrasonicDetectors::setup() {
  Serial.println("Setting up ultrasonic detectors...");
  for (int i = 0; i < count; i++) {
    detectors[i]->setup();
  }
  isEnabled = true;
  Serial.println("Ultrasonic detectors setup finished.");
}

void UltrasonicDetectors::disable() {
  isEnabled = false;
}

void UltrasonicDetectors::addDetector(int echoPin, int trigPin, int id) {
  Serial.print("[");
  Serial.print(echoPin);
  Serial.print("]");
  Serial.print("[");
  Serial.print(trigPin);
  Serial.print("]");
  Serial.println(" Adding ultrasonic detector...");
  detectors[count] = new UltrasonicDetector(echoPin, trigPin, id);
  detectors[count]->setup();
  count++;
  Serial.println("Ultrasonic detector added.");
}

void UltrasonicDetectors::clearDetectors() {
  for (int i = 0; i < count; i++) {
    delete detectors[i];
  }
  count = 0;
}

void UltrasonicDetectors::measure() {
  for (int i = 0; i < count; i++) {
    detectors[i]->measure();
  }
}

void UltrasonicDetectors::printResults() {
  for (int i = 0; i < count; i++) {
    detectors[i]->printResult();
  }
}

int UltrasonicDetectors::getCount() {
  return count;
}

int* UltrasonicDetectors::getDistances() {
  int* distances = new int[count];
  for (int i = 0; i < count; i++) {
    distances[i] = detectors[i]->getDistance();
  }
  return distances;
}

int UltrasonicDetectors::getDistance(int idx) {
    return detectors[idx]->getDistance();
}

bool UltrasonicDetectors::getIsEnabled() {
  return isEnabled;
}



enum BluetoothCommand {
    START_MEASUREMENT = 0,
    STOP_MEASUREMENT = 1,
    ENABLE_LIDAR = 2,
    DISABLE_LIDAR = 3,
    ENABLE_LUNA = 4,
    DISABLE_LUNA = 5,
    ENABLE_ULTRASONIC_DETECTORS = 6,
    DISABLE_ULTRASONIC_DETECTORS = 7
};



class ParkAssist {
  BLEServer* pServer;
  BLEService* pService;
  BLECharacteristic* pCharacteristic;
  BLEAdvertising* pAdvertising;

  Lidar* lidar;
  Luna* luna;
  UltrasonicDetectors* ultrasonic;

  CharacteristicCallbacks* characteristicCallbacks;
  ServerCallbacks* serverCallbacks;

  StaticJsonDocument<2048> jsonDocument;
  char jsonBuffer[2048];

  bool measurementEnabled;
  bool lidarEnabled;
  bool lunaEnabled;
  bool ultrasonicEnabled;

  void prepareResult();
  void sendResult();
  void setupSerial();
  void setupBluetooth();
  void startBluetoothAdvertising();
  void stopBluetoothAdvertising();

  public:
    ParkAssist();
    void setup();
    void onBluetoothConnected();
    void onBluetoothDisconnected();
    void onBluetoothCommand(std::string command);
    void run();
};

ParkAssist::ParkAssist() {
  measurementEnabled = false;

  lidar = new Lidar();
  luna = new Luna();
  ultrasonic = new UltrasonicDetectors();
  characteristicCallbacks = new CharacteristicCallbacks();
  serverCallbacks = new ServerCallbacks();
}

void ParkAssist::setup() {
  Serial.println("Setting up Park Assist...");
  setupSerial();
  setupBluetooth();
  startBluetoothAdvertising();
  Serial.println("Park Assist setup finished.");
}

void ParkAssist::onBluetoothConnected() {
  Serial.println("Bluetooth device connected.");
  stopBluetoothAdvertising();
}

void ParkAssist::onBluetoothDisconnected() {
  Serial.println("Bluetooth device disconnected.");
  measurementEnabled = false;
  if (lidar -> getIsEnabled()) {
    lidar -> stopMeasurement();
  }
  startBluetoothAdvertising();
}

void ParkAssist::setupSerial() {
  Serial.begin(9600);
  Serial.println("Serial setup finished.");
}

void ParkAssist::setupBluetooth() {
  Serial.println("Setting up bluetooth...");

  BLEDevice::init("Park Assist");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  pServer->setCallbacks(serverCallbacks);
  pCharacteristic->setCallbacks(characteristicCallbacks);

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();

  Serial.println("Bluetooth setup finished.");
}

void ParkAssist::startBluetoothAdvertising() {
  Serial.println("Starting bluetooth advertising...");
  pAdvertising->start();
  Serial.println("Bluetooth advertising started.");
}

void ParkAssist::stopBluetoothAdvertising() {
  Serial.println("Stopping bluetooth advertising...");
  pAdvertising->stop();
  Serial.println("Bluetooth advertising stopped.");
}

void ParkAssist::prepareResult() {
  jsonDocument.clear();

  if (lidar->getIsEnabled()) {
    JsonArray lidarMeasurementArray = jsonDocument.createNestedArray("multiPointLidar");
    int* lidarMeasurements = lidar->getMeasurements();
    for (int i = 0; i < 64; i++) {
      lidarMeasurementArray.add(lidarMeasurements[i]);
    }
  }

  if (luna->getIsEnabled()) {
    jsonDocument["singlePointLidar"] = luna->getDistance();
  }

  if (ultrasonic->getIsEnabled()) {
    JsonArray ultrasonicDetectorsArray = jsonDocument.createNestedArray("ultrasonic");
    for (int i = 0; i < ultrasonic->getCount(); i++) {
      ultrasonicDetectorsArray.add(ultrasonic->getDistance(i));
    }
  }


  serializeJson(jsonDocument, jsonBuffer);
  Serial.println(jsonBuffer);
}

void ParkAssist::sendResult() {
  pCharacteristic->setValue(jsonBuffer);
  pCharacteristic->notify();
}

void ParkAssist::onBluetoothCommand(std::string command) {
  jsonDocument.clear();
  deserializeJson(jsonDocument, command);

  BluetoothCommand bluetoothCommand = jsonDocument["command"];

  switch (bluetoothCommand) {
    case BluetoothCommand::START_MEASUREMENT: {
      Serial.println("Starting measurement...");
      measurementEnabled = true;
      if (lidar->getIsEnabled()) {
        lidar->startMeasurement();
      }
      Serial.println("Measurement started.");
      break;
    } case BluetoothCommand::STOP_MEASUREMENT: {
      Serial.println("Stopping measurement...");
      measurementEnabled = false;
      if (lidar->getIsEnabled()) {
        lidar->stopMeasurement();
      }
      Serial.println("Measurement stopped.");
      break;
    } case BluetoothCommand::ENABLE_LIDAR: {
      Serial.println("Enabling lidar...");
      lidar->setup();
      Serial.println("Lidar enabled.");
      break;
    } case BluetoothCommand::DISABLE_LIDAR: {
      Serial.println("Disabling lidar...");
      lidar->disable();
      Serial.println("Lidar disabled.");
      break;
    } case BluetoothCommand::ENABLE_LUNA: {
      Serial.println("Enabling Luna...");
      lunaEnabled = true;
      luna->setup();
      Serial.println("Luna enabled.");
      break;
    } case BluetoothCommand::DISABLE_LUNA: {
      Serial.println("Disabling Luna...");
      luna->disable();
      Serial.println("Luna disabled.");
      break;
    } case BluetoothCommand::ENABLE_ULTRASONIC_DETECTORS: {
      Serial.println("Enabling ultrasonic detectors...");
      ultrasonicEnabled = true;
      int detecorCount = jsonDocument["detectorCount"];

      ultrasonic->clearDetectors();
      for (int i = 0; i < detecorCount; i++) {
        int detectorIdx = jsonDocument["detectorIndices"][i];
        int detectorId = jsonDocument["detectorIds"][i];

        ultrasonic->addDetector(
          ECHO_PINS[detectorIdx],
          TRIG_PINS[detectorIdx],
          detectorId
        ); 
      }
      ultrasonic->setup();
      Serial.print("Ultrasonic detectors enabled. Count: ");
      Serial.println(detecorCount);
      break;
    } case BluetoothCommand::DISABLE_ULTRASONIC_DETECTORS: {
      Serial.println("Disabling ultrasonic detectors...");
      ultrasonic->disable();
      Serial.println("Ultrasonic detectors disabled.");
      break;
    }
  }
}


void ParkAssist::run() {
  if (measurementEnabled) {
    if (lidar->getIsEnabled()) {
      lidar->measure();
    }
    
    if (luna->getIsEnabled()) {
      luna->measure();
    }

    if (ultrasonic->getIsEnabled()) {
      ultrasonic->measure();
    }

    prepareResult();
    sendResult();
    delay(100);
  } else {
    delay(1000);
  }
}



ParkAssist* parkAssist = new ParkAssist();



void CharacteristicCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  parkAssist->onBluetoothCommand(pCharacteristic->getValue());
}

void ServerCallbacks::onConnect(BLEServer* pServer) {
  parkAssist->onBluetoothConnected();
}

void ServerCallbacks::onDisconnect(BLEServer *pServer) {
  parkAssist->onBluetoothDisconnected();
}



void setup() {
  parkAssist->setup();
}

void loop() {
  parkAssist->run();
}

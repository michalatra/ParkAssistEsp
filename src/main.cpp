#include <string>

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


// Defining bluetooth identifiers
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


// Setting up variables to control bluetooth connections
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristic;
BLEAdvertising* pAdvertising;


// Setting up display object
LiquidCrystal_I2C lcd(0x27,20,4);


// Program-level constants definitions
const int DETECTOR_PINOUT_COUNT = 4;
const int MAX_DETECTOR_COUNT = 8;

int ECHO_PINS[DETECTOR_PINOUT_COUNT] = {13, 4, 26, 33};
int TRIG_PINS[DETECTOR_PINOUT_COUNT] = {12, 27, 25, 32};

// Detectors data storage variables
int measuredDistances[MAX_DETECTOR_COUNT];
int measurementTime[MAX_DETECTOR_COUNT];
bool detectorStatus[MAX_DETECTOR_COUNT] = {false};


// Control variables declarations
int cableDetectorsCount = 0;
int wirelessDetectorsCount = 0;
bool measurementEnabled = false;


void setupCableDetectors(std::string value);
void startAdvertising();
void stopAdvertising();

// Defining callback for bluetooth transmission events
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      if (value == "START") {
        measurementEnabled = true;
      } else if (value == "STOP") {
        measurementEnabled = false;
      } else if (value == "WIRELESS_COUNT") {
        pCharacteristic->setValue(std::to_string(wirelessDetectorsCount));
      } else if (value == "ADD") {
        // TODO: Obłsuga dodawania bezprzewodowych czujników
      } else if (value == "REMOVE") {
        // TODO: Obsługa usuwania bezprzewodowych czujników
      } else if (value.find("SET_CABLE") != -1) {
        setupCableDetectors(value);
      }
    }
  }
};

// Defining callbacks for bluetooth connection events
class ServerCallbacks:  public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("New device connected");
    stopAdvertising();
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    measurementEnabled = false;
    startAdvertising();
  }
};


void setupSerial() {
  Serial.begin(9600);
}


// Setting up bluetooth connection and transmission parameters
void setupBluetooth() {
  Serial.println("Setting up bluetooth...");

  BLEDevice::init("Park Assist");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  pServer->setCallbacks(new ServerCallbacks());
  pCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pService->start();

  pAdvertising = BLEDevice::getAdvertising();

  Serial.println("Bluetooth setup finished.");

}


void setupCableDetectors(std::string value) {
  Serial.print("Setting up detectors placement: ");
  Serial.print(value.c_str());
  Serial.print("\n");

  for (int i=0; i < DETECTOR_PINOUT_COUNT; i++) {
    detectorStatus[i] = false;
  }

  for (int i=10; i < value.length(); i += 2) {
    Serial.println(value.at(i) - 48);
    detectorStatus[value.at(i) - 48] = true;
    cableDetectorsCount++;
  }

}


// Setting up cable detector pin outputs and inputs
void setupDetectorsPinouts() {
  Serial.println("Setting up detectors...");

  for (int i=0; i < DETECTOR_PINOUT_COUNT; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
  }

  Serial.println("Detectors setup finished.");
}


// Setting up screen properties
void setupScreen() {
  Serial.println("Setting up screen...");

  lcd.init();
  lcd.backlight();

  Serial.println("Screen setup finished.");
}


// Initializing bluetooth connection
void startAdvertising() {
  pAdvertising->start();
  Serial.println("Advertising initialized.");
}

void stopAdvertising() {
  pAdvertising->stop();
  Serial.println("Advertising finished.");
}

void printResultsOnScreen() {
  lcd.clear();
  int detectorCounter = 0;
  for (int i=0; i < MAX_DETECTOR_COUNT  && detectorCounter < 4; i++) {
    if (detectorStatus[i]) {
      lcd.setCursor((detectorCounter % 2) * 8, detectorCounter / 2);
      lcd.print(detectorCounter);
      lcd.print(": ");
      lcd.print(measuredDistances[i]);
      
      detectorCounter++;
    }
  }
}

std::string getStringifiedReadings() {
  std::string readings;
  int detectorCounter = 0;
  for (int i=0; i < MAX_DETECTOR_COUNT; i++) {
    if (detectorStatus[i]) {
      readings.append(std::to_string(measuredDistances[i]));
      readings.append(";");
      detectorCounter++;
    }
  }

  return readings;
}

void sendResultsViaBluetooth() {
  pCharacteristic->setValue(getStringifiedReadings());
}

void measureCableDetectors() {
  for (int i=0; i < DETECTOR_PINOUT_COUNT; i++) {
    if (detectorStatus[i]) {
      digitalWrite(TRIG_PINS[i], LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PINS[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PINS[i], LOW);
      digitalWrite(ECHO_PINS[i], HIGH);
      measuredDistances[i] = pulseIn(ECHO_PINS[i], HIGH) / 58;
    }
  }
}

void measureWirelessDetectors() {
  // TODO: Reading from connected wireless detectors
}

void setup() {
  setupSerial();
  setupBluetooth();
  setupDetectorsPinouts();
  setupScreen();
  startAdvertising();
}

void loop() {
  if (measurementEnabled) {
    measureCableDetectors();
    sendResultsViaBluetooth();
    printResultsOnScreen();
  } else {
    lcd.clear();
    lcd.println("Pomiar wstrzymany");
  }
  delay(500);
}
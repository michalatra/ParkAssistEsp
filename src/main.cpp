#include <string>

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

const int numberOfDetectors = 3;

int ECHO[numberOfDetectors] = {2, 5, 16};
int TRIG[numberOfDetectors] = {15, 17, 4};
int CM[numberOfDetectors];
int TIME[numberOfDetectors];

bool readDistance = false;

LiquidCrystal_I2C lcd(0x27,20,4);

class Callbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      if (value == "START") {
        readDistance = true;
      } else if (value == "STOP") {
        readDistance = false;
      } else if (value == "NUM") {
        pCharacteristic->setValue(std::to_string(numberOfDetectors));
      }

      Serial.println("**************");
      Serial.print("New value: ");
      for (int i=0; i<value.length(); i++) {
        Serial.print(value[i]);
      }
      Serial.println();
      Serial.println("**************");
    }
  }
};

void setupBluetooth() {
  BLEDevice::init("ESP32-BLE-Server");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new Callbacks());

  pCharacteristic->setValue("Hello world!");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
}

void setupDetectors() {
  for (int i=0; i < numberOfDetectors; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
  }
}

void setupScreen() {
  lcd.init();
  lcd.backlight();
}

void readSerialCommand() {
  std::string value = pCharacteristic->getValue();
  Serial.println(value.c_str());
}

void printResultsOnScreen() {
  lcd.clear();
  for (int i=0; i < numberOfDetectors; i++) {
    lcd.setCursor((i % 2) * 8, i / 2);
    lcd.print(i);
    lcd.print(": ");
    lcd.print(CM[i]);
  }
}

std::string getStringifiedReadings() {
  std::string readings;
  for (int i=0; i<numberOfDetectors; i++) {
    readings.append(std::to_string(CM[i]));
    readings.append(";");
  }

  return readings;
}

void sendResultsViaBluetooth() {
  pCharacteristic->setValue(getStringifiedReadings());
}

void calculateDistance() {
  for (int i=0; i < numberOfDetectors; i++) {
    digitalWrite(TRIG[i], LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG[i], LOW);
    digitalWrite(ECHO[i], HIGH); 
    TIME[i] = pulseIn(ECHO[i], HIGH);
    CM[i] = TIME[i] / 58;
  }
}

void setup() {
  Serial.begin(9600);
  setupBluetooth();
  setupDetectors();
  setupScreen();
}

void loop() {
  if (readDistance) {
    calculateDistance();
    sendResultsViaBluetooth();
    printResultsOnScreen();
  } else {
    lcd.clear();
    lcd.println("Pomiar wstrzymany");
  }
  delay(500);
}
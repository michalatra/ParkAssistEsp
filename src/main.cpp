#include <string>

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Defining bluetooth identifiers
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define RXD2 16
#define TXD2 17

// LI-DAR
int dist;
int strength;
float temperature;
unsigned char check;
int i;
unsigned char uart[9];
const int HEADER=0x59;
int rec_debug_state = 0x01;

bool lidar_enabled = false;
bool ultrasonic_enabled = false;
bool infrared_enabled = false;


// Setting up variables to control bluetooth connections
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristic;
BLEAdvertising* pAdvertising;


// Setting up display object
LiquidCrystal_I2C lcd(0x27,20,4);


// Program-level constants definitions
const int LIDAR_DETECTOR_COUNT = 2;
const int ULTRASONIC_DETECTOR__COUNT = 4;
const int INFRARED_DETECTOR__COUNT = 4;
const int MAX_DETECTOR_COUNT = 10;

int ECHO_PINS[ULTRASONIC_DETECTOR__COUNT] = {12, 27, 25, 32};
int TRIG_PINS[ULTRASONIC_DETECTOR__COUNT] = {13, 14, 26, 33};

// Detectors data storage variables
int measuredDistances[MAX_DETECTOR_COUNT] = {0};
bool detectorStatus[MAX_DETECTOR_COUNT] = {false};


// Control variables declarations
int cableDetectorsCount = 0;
int wirelessDetectorsCount = 0;
bool measurementEnabled = false;
bool screenEnabled = false;


void setupUltrasonicDetectors(std::string value);
void setupInfraredDetectors(std::string value);
void setupLidarDetectors(std::string value);
void startAdvertising();
void stopAdvertising();
void setupScreen();
void setupLidar();

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
      } else if (value == "ADD_WIRELESS") {
        // TODO: Obłsuga dodawania bezprzewodowych czujników
      } else if (value == "REMOVE_WIRELESS") {
        // TODO: Obsługa usuwania bezprzewodowych czujników
      } else if (value.find("SET_ULTRASONIC") != -1) {
        setupUltrasonicDetectors(value);
      } else if (value.find("SET_INFRARED") != -1) {
        // setupUltrasonicDetectors(value);
      } else if (value.find("SET_LIDAR") != -1) {
        setupLidarDetectors(value);
      } else if (value == "SCREEN") {
        setupScreen();
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


void setupUltrasonicDetectors(std::string value) {
  Serial.print("Setting up ultrasonic detectors placement: ");
  Serial.print(value.at(15));
  Serial.print("\n");

  for (int i=0; i < ULTRASONIC_DETECTOR__COUNT; i++) {
    detectorStatus[i + LIDAR_DETECTOR_COUNT] = false;
  }

  if (value.at(15) == 'F') {
    ultrasonic_enabled = false;
    return;
  }

  ultrasonic_enabled = true;

  for (int i=17; i < value.length(); i += 2) {
    Serial.println(value.at(i) - 48);
    detectorStatus[LIDAR_DETECTOR_COUNT + (value.at(i) - 48)] = true;
    cableDetectorsCount++;
  }

}

void setupInfraredDetectors(std::string value) {
  
  Serial.print("Setting up infrared detectors placement: ");
  Serial.print(value.c_str());
  Serial.print("\n");

  // TODO: Obsługa czujników podczerwonych
}

void setupLidarDetectors(std::string value) {
  
  Serial.print("Setting up lidar detectors: ");
  Serial.print(value.at(10));
  Serial.print("\n");

  for (int i=0; i < LIDAR_DETECTOR_COUNT; i++) {
    detectorStatus[i] = false;
  }

  if (value.at(10) == 'F') {
    lidar_enabled = false;
    return;
  }

  detectorStatus[0] = true;
  lidar_enabled = true;
  
}


// Setting up cable detector pin outputs and inputs
void setupDetectorsSockets() {
  Serial.println("Setting up detectors...");

  for (int i=0; i < ULTRASONIC_DETECTOR__COUNT; i++) {
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
  screenEnabled = true;

  Serial.println("Screen setup finished.");
}


// Setting up Lidar sensor
void setupLidar() {
  Serial.println("Setting up LiDAR...");

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.println("LiDAR setup finished.");
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

  Serial.print("Readings: ");
  Serial.print(readings.c_str());
  Serial.print("\n");

  return readings;
}

void sendResultsViaBluetooth() {
  Serial.println("Sending results");
  pCharacteristic->setValue(getStringifiedReadings());
}

void measureUltrasonicDetectors() {
  Serial.println("Reading data from wired detectors");

  for (int i=0; i < ULTRASONIC_DETECTOR__COUNT; i++) {
    if (detectorStatus[i + LIDAR_DETECTOR_COUNT]) {
      digitalWrite(TRIG_PINS[i], LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PINS[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PINS[i], LOW);
      digitalWrite(ECHO_PINS[i], HIGH);
      measuredDistances[i + LIDAR_DETECTOR_COUNT] = pulseIn(ECHO_PINS[i], HIGH, 100000) / 58;
      Serial.println(measuredDistances[i + LIDAR_DETECTOR_COUNT]);
    }
  }
}

void measureInfraredDetectors() {
  // TODO: Measuring infrared detectors
}

void measureWirelessDetectors() {
  // TODO: Reading from connected wireless detectors
}

void measureLidarDetectors() {
  while (rec_debug_state != 0x09) {
    if (Serial2.available()) {
      if (rec_debug_state == 0x01) {
        uart[0] = Serial2.read();
        if (uart[0] == 0x59) {
          check = uart[0];
          rec_debug_state = 0x02;
        }
      } else if(rec_debug_state == 0x02) {
        uart[1] = Serial2.read();
        if (uart[1] == 0x59) {
          check += uart[1];
          rec_debug_state = 0x03;
        } else {
          rec_debug_state = 0x01;
        }
      } else if (rec_debug_state == 0x03) {
        uart[2] = Serial2.read();
        check += uart[2];
        rec_debug_state = 0x04;
      } else if (rec_debug_state == 0x04) {
        uart[3] = Serial2.read();
        check += uart[3];
        rec_debug_state = 0x05;
      } else if (rec_debug_state == 0x05) {
        uart[4] = Serial2.read();
        check += uart[4];
        rec_debug_state = 0x06;
      } else if (rec_debug_state == 0x06) {
        uart[5] = Serial2.read();
        check += uart[5];
        rec_debug_state = 0x07;
      } else if (rec_debug_state == 0x07) {
        uart[6] = Serial2.read();
        check += uart[6];
        rec_debug_state = 0x08;
      } else if (rec_debug_state == 0x08) {
        uart[7] = Serial2.read();
        check += uart[7];
        rec_debug_state = 0x09;
      }
    }
  }

  uart[8] = Serial2.read();
  
  if (uart[8] == check) {
      dist = uart[2] + uart[3]*256;
      strength = uart[4] + uart[5]*256;
      temperature = uart[6] + uart[7] *256;
      temperature = temperature/8 - 256;                              
      Serial.print("dist = ");
      Serial.print(dist);
      Serial.print('\n');
      Serial.print("strength = ");
      Serial.print(strength);
      Serial.print('\n');
      Serial.print("\t Chip Temprature = ");
      Serial.print(temperature);
      Serial.println(" celcius degree");

      measuredDistances[0] = dist;     

      while(Serial2.available()) {
        Serial2.read();
      }



      delay(100);
  }

  rec_debug_state = 0x01;
}

void setup() {
  setupSerial();
  setupLidar();
  setupBluetooth();
  setupDetectorsSockets();
  startAdvertising();
}
void loop() {
  if (measurementEnabled) {
    if (lidar_enabled) {
      measureLidarDetectors();
    }

    if (ultrasonic_enabled) {
      measureUltrasonicDetectors();
    }

    if (infrared_enabled) {
      measureInfraredDetectors();
    }

    sendResultsViaBluetooth();
    if (screenEnabled) {
      printResultsOnScreen();
    }

    delay(200);
  } else {
    if (screenEnabled) {
      lcd.clear();
      lcd.println("Pomiar wstrzymany");
    }

    delay(1000);
  }
}
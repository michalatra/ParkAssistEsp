#include <Arduino.h>
#include "ArduinoJson.h"

#include <string>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define RXD2 16
#define TXD2 17

#define I2C_SDA 21
#define I2C_SCL 22

#define ULTRASONIC_DETECTOR__COUNT 4

const int TRIG_PINS[ULTRASONIC_DETECTOR__COUNT] = {12, 27, 25, 32};
const int ECHO_PINS[ULTRASONIC_DETECTOR__COUNT] = {13, 14, 26, 33};

enum BluetoothMessageField {
	COMMAND = 0,
	RESULT = 1,
	MULTI_POINT_LIDAR = 2,
	SINGLE_POINT_LIDAR = 3,
	ULTRASONIC = 4,
	DETECTOR_COUNT = 5,
	SOCKET_INDICES = 6,
	DETECTOR_IDS = 7,
};

const char *BLUETOOTH_MESSAGE_FIELD[] = {
	"command",
	"result",
	"multiPointLidar",
	"singlePointLidar",
	"ultrasonic",
	"detectorCount",
	"socketIndices",
	"detectorIds",
};

enum CommandResult {
	SUCCESS = 0,
	ERROR = 1,
	ACTION_NOT_NECESSARY = 2,
	I2C_ERROR = 3,
	LIDAR_ERROR = 4,
	LIDAR_RESOLUTION_ERROR = 5,
	LIDAR_RANGING_MODE_ERROR = 6,
	LIDAR_POWER_MODE_ERROR = 7,
	LIDAR_INTEGRATION_TIME_ERROR = 8,
	LIDAR_SHARPENER_ERROR = 9,
	LIDAR_TARGET_ORDER_ERROR = 10,
	LIDAR_RANGING_ERROR = 11,
	LIDAR_MEASUREMENT_ERROR = 12,
	LUNA_MEASUREMENT_ERROR = 13,
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic);
};

class ServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer *pServer);
	void onDisconnect(BLEServer *pServer);
};

class Lidar {
	SparkFun_VL53L5CX sensor;
	VL53L5CX_ResultsData measurement;

	int imageResolution = 0;
	int imageWidth = 0;
	bool i2cEnabled = false;
	bool rangingEnabled = false;
	bool lidarConnected = false;
	bool isEnabled = false;
	int measurements[64] = {0};

	CommandResult setupI2C();
	CommandResult disableI2C();
	CommandResult begin();
	CommandResult setupResolution();
	CommandResult setupRangingMode();
	CommandResult setPowerModeSleep();
	CommandResult setPowerModeWakeup();
	CommandResult setupIntegrationTime();
	CommandResult setupSharpener();
	CommandResult setupTargetOrder();

public:
	Lidar(){};
	CommandResult setup();
	CommandResult disable();
	CommandResult startMeasurement();
	CommandResult stopMeasurement();
	CommandResult measure();
	int *getMeasurements();
	bool getIsEnabled();
};

CommandResult Lidar::setupI2C() {
	Serial.println("Setting up I2C...");

	if (i2cEnabled) {
		Serial.println("I2C already enabled, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!Wire.begin() || !Wire.setClock(1000000)) {
		Serial.println("I2C setup failed.");
		i2cEnabled = false;
		return I2C_ERROR;
	}

	Serial.println("I2C setup finished.");
	i2cEnabled = true;
	return SUCCESS;
}

CommandResult Lidar::disableI2C() {
	Serial.println("Turning I2C off...");

	if (!i2cEnabled) {
		Serial.println("I2C already disabled, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!Wire.end()) {
		Serial.println("I2C disable failed.");
		return I2C_ERROR;
	}

	Serial.println("I2C disabled.");
	i2cEnabled = false;
	return SUCCESS;
}

CommandResult Lidar::begin() {
	Serial.println("Starting VL53L5CX...");

	if (lidarConnected) {
		Serial.println("VL53L5CX already connected, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.begin()) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	Serial.println("VL53L5CX started.");
	lidarConnected = true;
	return SUCCESS;
}

CommandResult Lidar::setupResolution() {
	Serial.println("Setting up resolution...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	imageResolution = sensor.getResolution();

	if (imageResolution == VL53L5CX_RESOLUTION_8X8) {
		Serial.println("VL53L5CX resolution is 8x8, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (sensor.setResolution(VL53L5CX_RESOLUTION_8X8)) {
		Serial.println("VL53L5CX resolution set to 8x8");
		imageResolution = sensor.getResolution();
		imageWidth = sqrt(imageResolution);
	}
	else {
		Serial.println("VL53L5CX failed to set resolution");
		return LIDAR_RESOLUTION_ERROR;
	}

	Serial.println("Resolution setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setupRangingMode() {
	Serial.println("Setting up ranging mode...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getRangingMode() == SF_VL53L5CX_RANGING_MODE::AUTONOMOUS) {
		Serial.println("VL53L5CX already in autonomous mode, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS)) {
		Serial.println(F("Failed to set autonomus ranging mode."));
		return LIDAR_RANGING_MODE_ERROR;
	}

	Serial.println("Ranging mode setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setPowerModeSleep() {
	Serial.println("Setting up power mode (sleep)...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getPowerMode() == SF_VL53L5CX_POWER_MODE::SLEEP) {
		Serial.println("VL53L5CX already in sleep mode, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP)) {
		Serial.println("VL53L5CX failed to set to sleep mode");
		return LIDAR_POWER_MODE_ERROR;
	}

	Serial.println("Power mode (sleep) setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setPowerModeWakeup() {
	Serial.println("Setting up power mode (wakeup)...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getPowerMode() == SF_VL53L5CX_POWER_MODE::WAKEUP) {
		Serial.println("VL53L5CX already in wakeup mode, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
		Serial.println("VL53L5CX failed to set to wakeup mode");
		return LIDAR_POWER_MODE_ERROR;
	}

	Serial.println("Power mode (wakeup) setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setupIntegrationTime() {
	Serial.println("Setting up integration time...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getIntegrationTime() == 100) {
		Serial.println("VL53L5CX integration time is 100ms, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setIntegrationTime(100)) {
		Serial.println("VL53L5CX failed to set integration time");
		return LIDAR_INTEGRATION_TIME_ERROR;
	}

	Serial.println("Integration time setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setupSharpener() {
	Serial.println("Setting up sharpener...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getSharpenerPercent() == 19) {
		Serial.println("VL53L5CX sharpener is 19 percent, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setSharpenerPercent(19)) {
		Serial.println("VL53L5CX failed to set sharpener");
		return LIDAR_SHARPENER_ERROR;
	}

	Serial.println("Sharpener setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setupTargetOrder() {
	Serial.println("Setting up target order...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (sensor.getTargetOrder() == SF_VL53L5CX_TARGET_ORDER::CLOSEST) {
		Serial.println("VL53L5CX target order is closest, no need to change");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST)) {
		Serial.println("VL53L5CX failed to set target order");
		return LIDAR_TARGET_ORDER_ERROR;
	}

	Serial.println("Target order setup finished.");
	return SUCCESS;
}

CommandResult Lidar::setup() {
	CommandResult lastCommandResult;

	lastCommandResult = setupI2C();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = begin();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = setupResolution();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = setupRangingMode();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = setupIntegrationTime();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = setupSharpener();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = setupTargetOrder();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	isEnabled = true;
	return SUCCESS;
}

CommandResult Lidar::disable() {
	CommandResult lastCommandResult;

	lastCommandResult = stopMeasurement();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	lastCommandResult = disableI2C();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	isEnabled = false;
	return SUCCESS;
}

CommandResult Lidar::startMeasurement() {
	Serial.println("Starting Lidar measurement...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (rangingEnabled) {
		Serial.println("VL53L5CX already ranging, no need to start");
		return ACTION_NOT_NECESSARY;
	}

	CommandResult lastCommandResult = setPowerModeWakeup();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	if (!sensor.startRanging()) {
		Serial.println("VL53L5CX failed to start ranging");
		return LIDAR_RANGING_ERROR;
	}

	Serial.println("VL53L5CX ranging started");
	rangingEnabled = true;
	return SUCCESS;
}

CommandResult Lidar::stopMeasurement() {
	Serial.println("Stopping Lidar measurement...");

	if (!lidarConnected) {
		Serial.println("VL53L5CX not connected, please check wiring");
		return LIDAR_ERROR;
	}

	if (!rangingEnabled) {
		Serial.println("VL53L5CX not ranging, no need to stop");
		return ACTION_NOT_NECESSARY;
	}

	if (!sensor.stopRanging()) {
		Serial.println("VL53L5CX failed to stop ranging");
		return LIDAR_RANGING_ERROR;
	}

	CommandResult lastCommandResult = setPowerModeSleep();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	Serial.println("VL53L5CX ranging stopped");
	rangingEnabled = false;
	return SUCCESS;
}

CommandResult Lidar::measure() {
	int idx = 0;
	if (!sensor.isDataReady())
		return LIDAR_MEASUREMENT_ERROR;
	if (!sensor.getRangingData(&measurement))
		return LIDAR_MEASUREMENT_ERROR;

	for (int y = 0; y < imageWidth * (imageWidth - 1); y += imageWidth) {
		for (int x = imageWidth - 1; x >= 0; x--) {
			measurements[idx] = measurement.distance_mm[y + x];
			idx++;
		}
	}

	return SUCCESS;
}

bool Lidar::getIsEnabled() {
	return isEnabled;
}

int *Lidar::getMeasurements() {
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

	CommandResult setupUart();
	CommandResult disableUart();
	void readUartMessage();
	void interpretMeasurementData();
	void printMeasurementData();

public:
	Luna(){};
	CommandResult setup();
	CommandResult disable();
	CommandResult measure();
	int getDistance();
	bool getIsEnabled();
};

CommandResult Luna::setupUart() {
	Serial.println("Setting up UART...");

	Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
	Serial.println("UART setup finished.");

	return SUCCESS;
}

CommandResult Luna::disableUart() {
	Serial.println("Disabling UART...");

	Serial2.end();
	Serial.println("UART disabled.");

	return SUCCESS;
}

CommandResult Luna::setup() {
	Serial.println("Setting up Luna LiDAR...");

	CommandResult lastCommandResult = setupUart();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	isEnabled = true;
	Serial.println("Luna setup finished.");
	return SUCCESS;
}

CommandResult Luna::disable() {
	Serial.println("Disabling Luna LiDAR...");

	CommandResult lastCommandResult = disableUart();
	if (lastCommandResult != SUCCESS && lastCommandResult != ACTION_NOT_NECESSARY)
		return lastCommandResult;

	isEnabled = false;
	Serial.println("Luna disabled.");
	return SUCCESS;
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
	distance = uartData[2] + uartData[3] * 256;
	signalStrength = uartData[4] + uartData[5] * 256;
	temperature = uartData[6] + uartData[7] * 256;
	temperature = temperature / 8 - 256;

	while (Serial2.available()) {
		Serial2.read();
	}

	delay(50);
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

CommandResult Luna::measure() {
	readUartMessage();

	if (uartData[8] != checksum)
		return LUNA_MEASUREMENT_ERROR;
	interpretMeasurementData();

	messageState = LunaMessageState::HEADER1;
	return SUCCESS;
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
	UltrasonicDetector *detectors[4];
	int count = 0;
	bool isEnabled = false;

public:
	UltrasonicDetectors(){};
	void setup();
	void disable();
	void addDetector(int echoPin, int trigPin, int id);
	void clearDetectors();
	void measure();
	void printResults();
	int getCount();
	int *getDistances();
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
	clearDetectors();
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

int *UltrasonicDetectors::getDistances() {
	int *distances = new int[count];
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
	BLEServer *pServer;
	BLEService *pService;
	BLECharacteristic *pCharacteristic;
	BLEAdvertising *pAdvertising;

	Lidar *lidar;
	Luna *luna;
	UltrasonicDetectors *ultrasonic;

	CharacteristicCallbacks *characteristicCallbacks;
	ServerCallbacks *serverCallbacks;

	StaticJsonDocument<2048> jsonDocument;
	char jsonBuffer[2048];

	bool measurementEnabled;
	bool lidarMeasured;
	bool lunaMeasured;
	bool ultrasonicMeasured;

	void prepareResult();
	void sendResult();
	void setupSerial();
	void setupBluetooth();
	void startBluetoothAdvertising();
	void stopBluetoothAdvertising();
	void sendCommandResult(BluetoothCommand command, CommandResult result);
	void startMeasurement();
	void stopMeasurement();
	void enableLidar();
	void disableLidar();
	void enableLuna();
	void disableLuna();
	void enableUltrasonicDetectors();
	void disableUltrasonicDetectors();

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
	lidarMeasured = false;
	lunaMeasured = false;
	ultrasonicMeasured = false;

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

	luna->disable();
	ultrasonic->disable();
	lidar->disable();

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
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

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

	if (lidarMeasured) {
		JsonArray lidarMeasurementArray = jsonDocument.createNestedArray(BLUETOOTH_MESSAGE_FIELD[MULTI_POINT_LIDAR]);
		int *lidarMeasurements = lidar->getMeasurements();
		for (int i = 0; i < 64; i++) {
			lidarMeasurementArray.add(lidarMeasurements[i]);
		}
	}

	if (lunaMeasured) {
		jsonDocument[BLUETOOTH_MESSAGE_FIELD[SINGLE_POINT_LIDAR]] = luna->getDistance();
	}

	if (ultrasonicMeasured) {
		JsonArray ultrasonicDetectorsArray = jsonDocument.createNestedArray(BLUETOOTH_MESSAGE_FIELD[ULTRASONIC]);
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

void ParkAssist::sendCommandResult(BluetoothCommand command, CommandResult result) {
	jsonDocument.clear();
	jsonDocument[BLUETOOTH_MESSAGE_FIELD[COMMAND]] = command;
	jsonDocument[BLUETOOTH_MESSAGE_FIELD[RESULT]] = result;
	serializeJson(jsonDocument, jsonBuffer);
	Serial.println(jsonBuffer);
	pCharacteristic->setValue(jsonBuffer);
	pCharacteristic->notify();
}

void ParkAssist::startMeasurement() {
	Serial.println("Starting measurement...");

	CommandResult lastCommandResult;

	if (!measurementEnabled) {
		if (lidar->getIsEnabled()) {
			lastCommandResult = lidar->startMeasurement();
			if (lastCommandResult == CommandResult::SUCCESS || lastCommandResult == CommandResult::ACTION_NOT_NECESSARY) {
				measurementEnabled = true;
			}
		} else {
			lastCommandResult = CommandResult::SUCCESS;
			measurementEnabled = true;
		}
	}

	sendCommandResult(BluetoothCommand::START_MEASUREMENT, lastCommandResult);

	if (measurementEnabled) {
		Serial.println("Measurement started.");
	} else {
		Serial.println("Measurement could not be started.");
	}
}

void ParkAssist::stopMeasurement() {
	Serial.println("Stopping measurement...");

	CommandResult lastCommandResult;

	if (measurementEnabled) {
		if (lidar->getIsEnabled()) {
			lastCommandResult = lidar->stopMeasurement();
			if (lastCommandResult == CommandResult::SUCCESS || lastCommandResult == CommandResult::ACTION_NOT_NECESSARY) {
				measurementEnabled = false;
			}
		} else {
			lastCommandResult = CommandResult::SUCCESS;
			measurementEnabled = false;
		}
	}

	sendCommandResult(BluetoothCommand::STOP_MEASUREMENT, lastCommandResult);

	if (measurementEnabled) {
		Serial.println("Measurement could not be stopped.");
	} else {
		Serial.println("Measurement stopped.");
	}
}

void ParkAssist::enableLidar() {
	Serial.println("Enabling lidar...");

	sendCommandResult(BluetoothCommand::ENABLE_LIDAR, lidar->setup());

	Serial.println("Lidar enabled.");
}

void ParkAssist::disableLidar() {
	Serial.println("Disabling lidar...");

	sendCommandResult(BluetoothCommand::DISABLE_LIDAR, lidar->disable());

	Serial.println("Lidar disabled.");
}

void ParkAssist::enableLuna() {
	Serial.println("Enabling luna...");

	sendCommandResult(BluetoothCommand::ENABLE_LUNA, luna->setup());

	Serial.println("Luna enabled.");
}

void ParkAssist::disableLuna() {
	Serial.println("Disabling luna...");

	sendCommandResult(BluetoothCommand::DISABLE_LUNA, luna->disable());

	Serial.println("Luna disabled.");
}

void ParkAssist::enableUltrasonicDetectors() {
	Serial.println("Enabling ultrasonic detectors...");
	int detecorCount = jsonDocument[BLUETOOTH_MESSAGE_FIELD[DETECTOR_COUNT]];

	ultrasonic->clearDetectors();

	for (int i = 0; i < detecorCount; i++) {
		int socketIdx = jsonDocument[BLUETOOTH_MESSAGE_FIELD[SOCKET_INDICES]][i];
		int detectorId = jsonDocument[BLUETOOTH_MESSAGE_FIELD[DETECTOR_IDS]][i];

		ultrasonic->addDetector(
			ECHO_PINS[socketIdx],
			TRIG_PINS[socketIdx],
			detectorId);
	}

	ultrasonic->setup();
	sendCommandResult(BluetoothCommand::ENABLE_ULTRASONIC_DETECTORS, SUCCESS);

	Serial.print("Ultrasonic detectors enabled. Count: ");
	Serial.println(detecorCount);
}

void ParkAssist::disableUltrasonicDetectors() {
	Serial.println("Disabling ultrasonic detectors...");

	ultrasonic->disable();
	sendCommandResult(BluetoothCommand::DISABLE_ULTRASONIC_DETECTORS, SUCCESS);

	Serial.println("Ultrasonic detectors disabled.");
}

void ParkAssist::onBluetoothCommand(std::string command) {
	jsonDocument.clear();
	deserializeJson(jsonDocument, command);

	BluetoothCommand bluetoothCommand = jsonDocument[BLUETOOTH_MESSAGE_FIELD[COMMAND]];

	switch (bluetoothCommand) {
		case BluetoothCommand::START_MEASUREMENT: {
			startMeasurement();
			break;
		}
		case BluetoothCommand::STOP_MEASUREMENT: {
			stopMeasurement();
			break;
		}
		case BluetoothCommand::ENABLE_LIDAR: {
			enableLidar();
			break;
		}
		case BluetoothCommand::DISABLE_LIDAR: {
			disableLidar();
			break;
		}
		case BluetoothCommand::ENABLE_LUNA: {
			enableLuna();
			break;
		}
		case BluetoothCommand::DISABLE_LUNA: {
			disableLuna();
			break;
		}
		case BluetoothCommand::ENABLE_ULTRASONIC_DETECTORS: {
			enableUltrasonicDetectors();
			break;
		}
		case BluetoothCommand::DISABLE_ULTRASONIC_DETECTORS: {
			disableUltrasonicDetectors();
			break;
		}
	}
}

void ParkAssist::run() {
	if (measurementEnabled) {
		if (lidar->getIsEnabled()) {
			lidarMeasured = lidar->measure();
		} else {
			lidarMeasured = false;
		}

		if (luna->getIsEnabled()) {
			lunaMeasured = luna->measure();
		} else {
			lunaMeasured = false;
		}

		if (ultrasonic->getIsEnabled()) {
			ultrasonic->measure();
			ultrasonicMeasured = true;
		} else {
			ultrasonicMeasured = false;
		}

		prepareResult();
		sendResult();
		delay(100);
	} else {
		delay(1000);
	}
}

ParkAssist *parkAssist = new ParkAssist();

void CharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
	std::string value = pCharacteristic->getValue();
	parkAssist->onBluetoothCommand(pCharacteristic->getValue());
}

void ServerCallbacks::onConnect(BLEServer *pServer) {
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

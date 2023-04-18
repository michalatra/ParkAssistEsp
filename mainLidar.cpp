#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData lidarMeasurement;

int imageResolution = 0;
int imageWidth = 0;

bool lidarAvailable = false;

void setupSerial() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Starting VL53L5CX Example");
}

void setupI2C() {
  // Wire.begin(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(1000000);
}

void setupResolution() {
  sensor.setResolution(VL53L5CX_RESOLUTION_8X8);
  imageResolution = sensor.getResolution();
  imageWidth = sqrt(imageResolution);

  if (sensor.isConnected()) {
    Serial.println("VL53L5CX is connected");
  } else {
    Serial.println("VL53L5CX is not connected");
    lidarAvailable = false;
  }
}

void setupRagingMode() {
  if (sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS)) {
    Serial.println("VL53L5CX set to autonomous mode");
    
    SF_VL53L5CX_RANGING_MODE mode = sensor.getRangingMode();
    
    switch (mode) {
      case SF_VL53L5CX_RANGING_MODE::AUTONOMOUS:
        Serial.println(F("Ranging mode set to autonomous."));
        break;

      case SF_VL53L5CX_RANGING_MODE::CONTINUOUS:
        Serial.println(F("Ranging mode set to continuous."));
        break;

      default:
        Serial.println(F("Error recovering ranging mode."));
        break;
    }
  } else {
    Serial.println("VL53L5CX failed to set to autonomous mode");
    lidarAvailable = false;
  }
}

void setupPowerModeSleep() {
  if  (sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP)) {
    Serial.println("VL53L5CX set to sleep mode");
  } else {
    Serial.println("VL53L5CX failed to set to sleep mode");
    lidarAvailable = false;
  }

   SF_VL53L5CX_POWER_MODE currentPowerMode = sensor.getPowerMode();

  switch (currentPowerMode) {
    case SF_VL53L5CX_POWER_MODE::SLEEP:
      Serial.println(F("Shhhh... device is sleeping!"));
      break;

    case SF_VL53L5CX_POWER_MODE::WAKEUP:
      Serial.println(F("Device is awake."));
      break;

    default:
      Serial.println(F("Cannot retrieve device power mode."));
      break;
  }
}

void setupPowerModeWakeup() {
  Serial.print(F("Waking up device in 5..."));
  delay(1000);
  Serial.print(F(" 4..."));
  delay(1000);
  Serial.print(F(" 3..."));
  delay(1000);
  Serial.print(F(" 2..."));
  delay(1000);
  Serial.println(F(" 1..."));
  delay(1000);

  if (sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
    Serial.println("VL53L5CX set to wakeup mode");
  } else {
    Serial.println("VL53L5CX failed to set to wakeup mode");
    lidarAvailable = false;
  }

  switch (sensor.getPowerMode()) {
    case SF_VL53L5CX_POWER_MODE::SLEEP:
      Serial.println(F("Shhhh... device is sleeping!"));
      break;

    case SF_VL53L5CX_POWER_MODE::WAKEUP:
      Serial.println(F("Device is awake."));
      break;

    default:
      Serial.println(F("Cannot retrieve device power mode."));
      break;
  }
}

void setupIntegrationTime() {
  Serial.print(F("Current integration time: "));
  Serial.print(sensor.getIntegrationTime());
  Serial.println(F("ms"));

  if (sensor.setIntegrationTime(100)) {
    Serial.print(F("Current integration time: "));
    Serial.print(sensor.getIntegrationTime());
    Serial.println(F("ms"));
  } else {
    Serial.println(F("Failed to set integration time."));
    lidarAvailable = false;
  }
}

void setupSharpener() {
  Serial.print(F("Current sharpener value: "));
  Serial.print(sensor.getSharpenerPercent());
  Serial.println(F("%"));

  if (sensor.setSharpenerPercent(19)) {
    Serial.print(F("Current sharpener value: "));
    Serial.print(sensor.getSharpenerPercent());
    Serial.println(F("%"));
  } else {
    Serial.println(F("Failed to set sharpener value."));
    lidarAvailable = false;
  }
}

void setupTargetOrder() {
  SF_VL53L5CX_TARGET_ORDER order = sensor.getTargetOrder();

  switch (order) {
    case SF_VL53L5CX_TARGET_ORDER::STRONGEST:
      Serial.println(F("Current target order is strongest."));
      break;

    case SF_VL53L5CX_TARGET_ORDER::CLOSEST:
      Serial.println(F("Current target order is closest."));
      break;

    default:
      Serial.println(F("Cannot get target order."));
      break;
  }

  if (sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST)) {
    order = sensor.getTargetOrder();
    switch (order)
    {
      case SF_VL53L5CX_TARGET_ORDER::STRONGEST:
        Serial.println(F("Target order set to strongest."));
        break;

      case SF_VL53L5CX_TARGET_ORDER::CLOSEST:
        Serial.println(F("Target order set to closest."));
        break;

      default:
        break;
    }
  } else {
    Serial.println(F("Failed to set target order."));
    lidarAvailable = false;
  }
}

void setupRanging() {
  if (sensor.startRanging()) {
    Serial.println(F("Ranging started."));
  } else {
    Serial.println(F("Failed to start ranging."));
    lidarAvailable = false;
  }
}

void setupLidar() {
  lidarAvailable = sensor.begin();
  
  if (!lidarAvailable) return;

  if (lidarAvailable) setupResolution();
  if (lidarAvailable) setupRagingMode();
  if (lidarAvailable) setupPowerModeSleep();
  if (lidarAvailable) setupPowerModeWakeup();
  if (lidarAvailable) setupIntegrationTime();
  if (lidarAvailable) setupSharpener();
  if (lidarAvailable) setupTargetOrder();
  if (lidarAvailable) setupRanging();
}


void setup() {
  setupSerial();
  setupI2C();
  setupLidar();
}

void loop() {
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&lidarMeasurement)) {
      for (int y = 0; y < imageWidth * (imageWidth - 1); y += imageWidth) {
        for (int x = imageWidth - 1; x >= 0; x--) {
          Serial.print("\t");
          Serial.print(lidarMeasurement.distance_mm[y + x]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5);
}
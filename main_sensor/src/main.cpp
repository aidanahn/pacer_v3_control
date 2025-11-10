#include <Arduino.h>
#include <QTRSensors.h>

// UART Configuration
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 115200

// Hardware Configuration
const uint8_t SENSOR_COUNT = 16;
const uint8_t SENSOR_PINS[] = {32, 33, 26, 27, 23, 25, 21, 22, 18, 19, 14, 15, 5, 13, 2, 4};

// Global Objects
QTRSensors qtr;

// Global Variables
uint16_t sensorValues[SENSOR_COUNT];
bool isPacing = false;
String lineMode = "BLACK_LINE";
uint16_t position = 0;

// Function Declarations
void flashLED(uint8_t times, uint16_t duration);
void calibrateSensor();

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  qtr.setTypeRC();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
  qtr.setDimmingLevel(0);
  qtr.setTimeout(3000);
  qtr.resetCalibration();

  pinMode(LED_BUILTIN, OUTPUT);
  flashLED(3, 250);
}

void loop() {
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();

    if (command == "START_PACING") {
      isPacing = true;
    }
    else if (command == "STOP_PACING") {
      isPacing = false;
    }
    else if (command == "START_CALIBRATION") {
      calibrateSensor();
      Serial2.println("STOP_CALIBRATION");
    }
    else if (command.startsWith("SET_LINE_MODE ")) {
      lineMode = command.substring(strlen("SET_LINE_MODE "));
      lineMode.trim();
    }
  }

  if (isPacing) {
    if (lineMode == "BLACK_LINE") {
      position = qtr.readLineBlack(sensorValues);
    } else {
      position = qtr.readLineWhite(sensorValues);
    }
    Serial2.println(position);
  }
}

void flashLED(uint8_t times, uint16_t duration) {
  for (uint8_t i = 0; i < times; i++) {
    delay(duration);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(duration);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void calibrateSensor() {
  qtr.resetCalibration();
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}
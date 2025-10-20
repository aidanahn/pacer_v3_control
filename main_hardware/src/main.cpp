#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <AS5600.h>
#include <PID_v1.h>

// UART Configuration
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 115200

// Hardware Configuration
const int ESC_LED_CHANNEL = 0;
const int SERVO_LED_CHANNEL = 1;
const int FREQUENCY = 50;
const int RESOLUTION = 16;
const int MAX_THROTTLE = 2000;
const int NEUTRAL_THROTTLE = 1500;
const int MIN_THROTTLE = 1000;

// Global Objects
WebServer server(80);
AS5600 as5600;

// Global Variables
bool isPacing = false;
int paceDistance = 0;
int paceMicroseconds = NEUTRAL_THROTTLE;
String lineMode = "BLACK_LINE";

// Distance Calculation Constants
const double GEAR_RATIO = (40.0 / 20.0) * (38.0 / 13.0);
const double WHEEL_CIRCUMFERENCE = PI * (121.9 / 1000.0);

// PID Constants
double servoKp = 0.0;
double servoKd = 0.0;
double servoKi = 0.0;
double ServoSetpoint, ServoInput, ServoOutput;
PID ServoPID(&ServoInput, &ServoOutput, &ServoSetpoint, servoKp, servoKi, servoKd, REVERSE);

// Pin Definitions
#define ESC_PIN 32
#define SERVO_PIN 33

// WiFi Configuration
const char* SSID = "PacerESP32";
const char* PASSWORD = "04082008";

// Function Declarations
void flashLED(uint8_t times, uint16_t duration);
void setESCMicroseconds(int microseconds);
void setServoAngle(int angle);
void handleRoot();
void handleStart();
void handleStop();
void handlePID();
void handleCalibrate();
void handleLine();
double getDistanceTraveled();

void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  Wire.begin();

  ledcSetup(ESC_LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(ESC_PIN, ESC_LED_CHANNEL);
  ledcSetup(SERVO_LED_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_LED_CHANNEL);

  WiFi.softAP(SSID, PASSWORD);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/pid", HTTP_POST, handlePID);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/line", HTTP_POST, handleLine);
  server.begin();

  ServoSetpoint = 7500;
  ServoPID.SetSampleTime(20);
  ServoPID.SetOutputLimits(-90, 90);
  ServoPID.SetMode(AUTOMATIC);

  setESCMicroseconds(NEUTRAL_THROTTLE);
  setServoAngle(90);
  as5600.resetCumulativePosition(0);

  pinMode(LED_BUILTIN, OUTPUT);
  flashLED(3, 250);
}

void loop() {
  server.handleClient();

  if (isPacing) {
    if (Serial2.available()) {
      String position = Serial2.readStringUntil('\n');
      position.trim();
      ServoInput = position.toInt();
      ServoPID.Compute();
      setServoAngle(90 + (int)ServoOutput);
    }
    if (getDistanceTraveled() >= (double)paceDistance) {
      handleStop();
    }
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

void setESCMicroseconds(int microseconds) {
  microseconds = constrain(microseconds, 1000, 2000);
  ledcWrite(ESC_LED_CHANNEL, (microseconds * 65535) / 20000);
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulse_us = map(angle, 0, 180, 1000, 2000);
  ledcWrite(SERVO_LED_CHANNEL, (pulse_us * 65535) / 20000);
}

void handleRoot() {
  String html = "";

  html += "<!DOCTYPE html><html><body>";

  html += "<form action='/start' method='POST'>";
  html += "<label for='paceDistance'>Distance (meters): </label>";
  html += "<input type='number' id='paceDistance' name='paceDistance' value='" + String(paceDistance) + "'><br>";
  html += "<label for='paceMicroseconds'>Microseconds (>1500-2000): </label>";
  html += "<input type='number' id='paceMicroseconds' name='paceMicroseconds' value='" + String(paceMicroseconds) + "'><br>";
  html += "<button type='submit'>Start Pacing</button>";
  html += "</form>";

  html += "<form action='/stop' method='POST'>";
  html += "<button type='submit'>Stop Pacing</button>";
  html += "</form>";

  html += "<form action='/line' method='POST'>";
  html += "<select id='lineMode' name='lineMode'>";
  html += String("<option value='BLACK_LINE'") + (lineMode == "BLACK_LINE" ? " selected" : "") + ">Black Line</option>";
  html += String("<option value='WHITE_LINE'") + (lineMode == "WHITE_LINE" ? " selected" : "") + ">White Line</option>";
  html += "</select> ";
  html += "<button type='submit'>Update</button>";
  html += "</form>";

  html += "<form action='/pid' method='POST'>";
  html += "<label for='servoKp'>Proportional (Kp): </label>";
  html += "<input type='number' id='servoKp' name='servoKp' step='0.001' value='" + String(servoKp, 3) + "'><br>";
  html += "<label for='servoKd'>Derivative (Kd): </label>";
  html += "<input type='number' id='servoKd' name='servoKd' step='0.001' value='" + String(servoKd, 3) + "'><br>";
  html += "<label for='servoKi'>Integral (Ki): </label>";
  html += "<input type='number' id='servoKi' name='servoKi' step='0.001' value='" + String(servoKi, 3) + "'><br>";
  html += "<button type='submit'>Update PID</button>";
  html += "</form>";

  html += "<form action='/calibrate' method='POST'>";
  html += "<button type='submit'>Calibrate Sensor</button>";
  html += "</form>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleStart() {
  if (server.hasArg("paceDistance")) paceDistance = server.arg("paceDistance").toInt();
  if (server.hasArg("paceMicroseconds")) paceMicroseconds = server.arg("paceMicroseconds").toInt();

  Serial2.println("START_PACING");
  
  isPacing = true;
  setServoAngle(90);
  as5600.resetCumulativePosition(0);
  setESCMicroseconds(paceMicroseconds);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  Serial2.println("STOP_PACING");
  isPacing = false;
  setServoAngle(90);
  as5600.resetCumulativePosition(0);
  setESCMicroseconds(NEUTRAL_THROTTLE);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handlePID() {
  if (server.hasArg("servoKp")) servoKp = server.arg("servoKp").toDouble();
  if (server.hasArg("servoKd")) servoKd = server.arg("servoKd").toDouble();
  if (server.hasArg("servoKi")) servoKi = server.arg("servoKi").toDouble();
  ServoPID.SetTunings(servoKp, servoKi, servoKd);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleCalibrate() {
  handleStop();
  Serial2.println("START_CALIBRATION");
  while (true) {
    if (Serial2.available()) {
      String response = Serial2.readStringUntil('\n');
      response.trim();
      if (response == "STOP_CALIBRATION") {
        break;
      }
    }
  }

  server.sendHeader("Location", "/");
  server.send(303);

  flashLED(5, 75);
}

void handleLine() {
  handleStop();
  if (server.hasArg("lineMode")) {
    lineMode = server.arg("lineMode");
    Serial2.println("SET_LINE_MODE " + lineMode);
  }

  server.sendHeader("Location", "/");
  server.send(303);

  flashLED(5, 75);
}

double getDistanceTraveled() {
  return abs(as5600.getCumulativePosition() / 4096.0 / GEAR_RATIO * WHEEL_CIRCUMFERENCE);
}
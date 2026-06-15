#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>

/*
* ESP32 + 28BYJ-48 + ULN2003 + VL53L0X (Pololu library)
* ------------------------------------------------------
* - FULL4WIRE (max torque)
* - SDA = 33, SCL = 32

Page 10 of 14

* - Default: 7 RPM, reverse every 360°
* - Outputs ONLY: "angle,distance,"
* - Works directly with your Processing radar sketch (no changes needed)
*/

// -------- SPECIFIED SETTINGS --------
const float RPM_DEFAULT = 35.0; // motor speed
const float STOP_ANGLE_DEG = 360.0; // reverse angle
const long STEPS_PER_REV = 2048;
const uint16_t SAMPLE_MS = 25; // sensor sample period (25–35 ms typical)
// --------------------------------

// Stepper pins: (IN1, IN3, IN2, IN4)
AccelStepper stepper(AccelStepper::FULL4WIRE, 14, 26, 27, 25);
VL53L0X sensor;

inline float stepsPerSecond(float rpm) { return (rpm * STEPS_PER_REV) / 60.0f; }
inline long degToSteps(float deg) { return (long)((deg / 360.0f) * STEPS_PER_REV + 0.5f); }
inline float stepsToDeg(long steps) { return (steps * 360.0f) / (float)STEPS_PER_REV; }

long segStart = 0;
int dirSign = +1;

uint16_t readDistanceMM() {
uint16_t d = sensor.readRangeSingleMillimeters();
if (sensor.ƟmeoutOccurred()) return 0;
return d;
}

// as we want to print exactly "angle,distance\n"
void printAngleDistanceInt(int angleDeg) {

Page 11 of 14

uint16_t d = readDistanceMM();
Serial.print(angleDeg);
Serial.print(',');
Serial.println(d);
Serial.print(',');
}

void startNewLeg() {
segStart = stepper.currentPosiƟon();
printAngleDistanceInt(0); // to obtain exact 0° reading (Processing clears here)
long delta = dirSign * degToSteps(STOP_ANGLE_DEG);
stepper.moveTo(segStart + delta);
}

void setup() {
Serial.begin(9600); // match Processing baud rate
Wire.begin(32, 33); // <-- SDA and SCL pins
sensor.setTimeout(500);
sensor.init();

stepper.setAcceleraƟon(200.0);
float sps = stepsPerSecond(RPM_DEFAULT);
stepper.setMaxSpeed(sps);
stepper.setSpeed(sps);

startNewLeg();
}
void loop() {
stepper.run();
// ConƟnuous sampling while rotaƟng
staƟc unsigned long lastSample = 0;

Page 12 of 14

unsigned long now = millis();
if (now - lastSample >= SAMPLE_MS) {
lastSample = now;
long stepsSince = labs(stepper.currentPosiƟon() - segStart);
float angleF = stepsToDeg(stepsSince);
if (angleF < 0) angleF = 0;
if (angleF > STOP_ANGLE_DEG) angleF = STOP_ANGLE_DEG;
int angleInt = (int)(angleF + 0.5f);
if (angleInt > 0 && angleInt < (int)STOP_ANGLE_DEG) {
printAngleDistanceInt(angleInt);
}
}
// End of sweep: reverse direcƟon and restart
if (stepper.distanceToGo() == 0) {
printAngleDistanceInt((int)STOP_ANGLE_DEG);
dirSign = -dirSign;
float sps = stepsPerSecond(RPM_DEFAULT);
stepper.setSpeed(dirSign > 0 ? sps : -sps);
startNewLeg();
delay(120);
}}
#include <Wire.h>
#include <AS5600.h>
#include "Adafruit_TinyUSB.h"

#define EnClutch 0 // Set to 1 to enable the Clutch pedal

// --- PIN ASSIGNMENTS ---
// IBT-2 Motor Driver Pins
const int RPWM_PIN = 18;  // Connect to IBT-2 RPWM
const int LPWM_PIN = 19;  // Connect to IBT-2 LPWM

// Pedal Pins (Must use ADC-capable pins: 26, 27, or 28)
const int ACCEL_PIN  = 26; // ADC0
const int BRAKE_PIN  = 27; // ADC1
const int CLUTCH_PIN = 28; // ADC2

// I2C Pins for AS5600 Encoder
const int SDA_PIN = 20;
const int SCL_PIN = 21;

// --- OBJECTS & GLOBALS ---
AS5600 as5600;
Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp = { 0 };

long totalDegrees = 0;
int lastRawAngle = 0;

// --- TUNING PARAMETERS ---
int springStrength = 3;     // Increase for a stiffer "return to center"
float gearRatioMultiplier = 1.0; // Adjust if using a belt/gear drive
int loopDelay = 5;          // 5ms = 200Hz update rate

// Xbox-style HID Report Descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(1))
};

void setup() {
  // 1. Initialize PWM Pins for IBT-2
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  // 2. Initialize I2C for AS5600
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  as5600.begin();
  
  // 3. Initialize USB HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // 4. Set Initial Position
  lastRawAngle = as5600.readAngle();

  // Wait for USB to be ready before starting loop
  while( !TinyUSBDevice.mounted() ) delay(1);
}

void loop() {
  if (!TinyUSBDevice.mounted()) return;

  // --- STEP 1: HANDLE PEDAL INPUTS ---
  // Pico ADC is 12-bit (0-4095). We map this to 8-bit HID (-127 to 127).
  int8_t AccelVal = map(analogRead(ACCEL_PIN), 0, 4095, -127, 127);
  int8_t BrakeVal = map(analogRead(BRAKE_PIN), 0, 4095, -127, 127);
  int8_t ClutchVal = 0;
  
  if (EnClutch) {
    ClutchVal = map(analogRead(CLUTCH_PIN), 0, 4095, -127, 127);
  }

  // --- STEP 2: HANDLE STEERING ROTATION ---
  int currentRawAngle = as5600.readAngle();
  int diff = currentRawAngle - lastRawAngle;

  // Handle magnetic encoder wrap-around (4096 steps per rotation)
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;
  
  totalDegrees += (diff * gearRatioMultiplier);
  lastRawAngle = currentRawAngle;

  // Map rotation to HID range (approx 900 degrees total lock-to-lock)
  long constrainedDegrees = constrain(totalDegrees, -5120, 5120); 
  int16_t mappedSteering = map(constrainedDegrees, -5120, 5120, -127, 127);
  
  // --- STEP 3: SEND HID DATA TO PC ---
  gp.x  = mappedSteering; // Steering Axis
  gp.y  = AccelVal;       // Accelerator Axis
  gp.z  = BrakeVal;       // Brake Axis
  gp.rz = ClutchVal;      // Clutch Axis
  
  usb_hid.sendReport(1, &gp, sizeof(gp));

  // --- STEP 4: CENTERING FORCE (Basic Spring) ---
  long error = 0 - totalDegrees; 
  int16_t centeringForce = error * springStrength;
  centeringForce = constrain(centeringForce, -255, 255);

  if (centeringForce > 40) {
    // Turning Right Force
    analogWrite(RPWM_PIN, centeringForce / 5);
    analogWrite(LPWM_PIN, 0);
  } 
  else if (centeringForce < -40) {
    // Turning Left Force
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, abs(centeringForce / 5));
  } 
  else {
    // Deadzone - no motor movement
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
  }
  
  delay(loopDelay); 
}

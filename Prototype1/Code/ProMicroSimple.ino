#include <Wire.h>
#include <AS5600.h>
#include <Joystick.h>

#define EnClutch 0 // Set to 1 to enable Clutch

// IBT-2 Pins for PWM (Pro Micro compatible pins)
const int RPWM_PIN = 9;  
const int LPWM_PIN = 10; 

// Pedal Pins (Analog)
const int ACCEL_PIN = A0; // Pin 18
const int BRAKE_PIN = A1; // Pin 19
const int CLUTCH_PIN = A2; // Pin 20

AS5600 as5600;

// Initialize Joystick
// Parameters: ID, Type, Button Count, Hat Count, X, Y, Z, Rx, Ry, Rz, Rudder, Throttle, Accelerator, Brake, Steering
Joystick_ Joystick(0x04, JOYSTICK_TYPE_GAMEPAD, 
  0, 0,        // Buttons, Hat
  true, true,  // X (Steering), Y (Accelerator)
  true, false, // Z (Brake), Rx
  false, false,// Ry, Rz
  false, false,// Rudder, Throttle
  false, false, false); // Accelerator, Brake, Steering (custom labels)

long totalDegrees = 0;
int lastRawAngle = 0;

// Force Feedback Tuning
int springStrength = 3; 
float gearRatioMultiplier = 1.0; 

void setup() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  
  // Pro Micro Hardware I2C: Pins 2 (SDA) and 3 (SCL)
  Wire.begin();
  as5600.begin();
  
  Joystick.begin();
  
  // Set Ranges for HID
  Joystick.setXAxisRange(-127, 127);
  Joystick.setYAxisRange(-127, 127);
  Joystick.setZAxisRange(-127, 127);

  lastRawAngle = as5600.readAngle();
}

void loop() {
  // --- 1. HANDLE PEDAL INPUT ---
  int Accel = analogRead(ACCEL_PIN);
  int Brake = analogRead(BRAKE_PIN);
  int Clutch = 0;
  if (EnClutch) {
    Clutch = analogRead(CLUTCH_PIN);      
  }

  // Map 10-bit (0-1023) to HID range (-127 to 127)
  int8_t AccelVal = map(Accel, 0, 1023, -127, 127);
  int8_t BrakeVal = map(Brake, 0, 1023, -127, 127);

  // --- 2. HANDLE STEERING INPUT ---
  int currentRawAngle = as5600.readAngle();
  int diff = currentRawAngle - lastRawAngle;

  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;
  
  totalDegrees += (diff * gearRatioMultiplier);
  lastRawAngle = currentRawAngle;

  // Map steering to HID
  long constrainedDegrees = constrain(totalDegrees, -5120, 5120); 
  int16_t mappedSteering = map(constrainedDegrees, -5120, 5120, -127, 127);
  
  // --- 3. UPDATE HID REPORT ---
  Joystick.setXAxis(mappedSteering);
  Joystick.setYAxis(AccelVal);
  Joystick.setZAxis(BrakeVal);

  // --- 4. CENTERING FORCE (Spring) ---
  long error = 0 - totalDegrees; 
  int16_t centeringForce = error * springStrength;
  centeringForce = constrain(centeringForce, -255, 255);

  if (centeringForce > 40) {
    analogWrite(RPWM_PIN, centeringForce / 5);
    analogWrite(LPWM_PIN, 0);
  } else if (centeringForce < -40) { // Fixed: threshold for opposite direction
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, abs(centeringForce / 5));
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
  }
  
  delay(10); 
}

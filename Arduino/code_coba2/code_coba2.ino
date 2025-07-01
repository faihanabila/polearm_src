#include <Wire.h>

// Pin definitions
#define STEP_PIN 3
#define DIR_PIN 2

// AS5600 I2C address and registers
#define AS5600_ADDRESS 0x36
#define AS5600_ANGLE_REG 0x0E  // Raw angle register (12-bit)

// Motor parameters
#define STEPS_PER_REV 200      // Standard NEMA 23 (1.8° per step)
#define MICROSTEPS 16          // Adjust based on your driver settings
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)
#define STEP_DELAY 1000        // Increased delay for more reliable stepping

// Variables
float currentPosition = 0.0;   // Current position in degrees from AS5600
float startPosition = 0.0;     // Starting position for relative moves
float targetPosition = 0.0;    // Target position in degrees
String inputString = "";       // String to hold incoming data
bool stringComplete = false;   // Flag for complete string
unsigned long lastEncoderRead = 0;
const unsigned long ENCODER_INTERVAL = 100; // Read encoder every 100ms

void setup() {
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("NEMA 23 Stepper Motor Calibration with AS5600 Encoder");
  Serial.println("Commands:");
  Serial.println("  'cal360' - Rotate motor 360° and measure encoder steps");
  Serial.println("  'pos' - Get current encoder position");
  Serial.println("  'steps <number>' - Move specified number of motor steps");
  Serial.println("Ready for calibration...");
  
  // Read initial encoder position
  delay(100);
  readAS5600();
}

void loop() {
  // Read encoder position periodically for real-time feedback
  if (millis() - lastEncoderRead >= ENCODER_INTERVAL) {
    readAS5600();
    lastEncoderRead = millis();
  }
  
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  if (command == "cal360") {
    Serial.println("Starting 360° calibration...");
    calibrate360();
  }
  else if (command.startsWith("steps ")) {
    int steps = command.substring(6).toInt();
    if (steps > 0) {
      Serial.println("Moving " + String(steps) + " motor steps...");
      moveSteps(steps, true); // Default clockwise
    } else if (steps < 0) {
      Serial.println("Moving " + String(abs(steps)) + " motor steps CCW...");
      moveSteps(abs(steps), false);
    } else {
      Serial.println("Invalid step count");
    }
  }
  else if (command == "pos") {
    readAS5600();
    Serial.println("Current encoder position: " + String(currentPosition, 2) + "°");
  }
  else {
    Serial.println("Unknown command. Use: cal360, steps <number>, or pos");
  }
}

void calibrate360() {
  // Read starting position
  readAS5600();
  float startPos = currentPosition;
  Serial.println("Starting position: " + String(startPos, 2) + "°");
  
  // Calculate total motor steps for 360°
  int totalMotorSteps = TOTAL_STEPS;
  Serial.println("Motor steps for 360°: " + String(totalMotorSteps));
  
  // Set direction (clockwise)
  digitalWrite(DIR_PIN, HIGH);
  
  // Move motor 360° and track encoder
  Serial.println("Rotating motor 360° clockwise...");
  
  float encoderReadings[21]; // Store 21 readings (every 5% of rotation)
  int readingIndex = 0;
  
  for (int step = 0; step <= totalMotorSteps; step++) {
    // Take a step (except on first iteration)
    if (step > 0) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY);
    }
    
    // Read encoder at specific intervals
    if (step % (totalMotorSteps / 20) == 0 || step == totalMotorSteps) {
      readAS5600();
      encoderReadings[readingIndex] = currentPosition;
      
      float percentComplete = (float)step / totalMotorSteps * 100.0;
      float encoderChange = currentPosition - startPos;
      
      // Handle 360° wraparound
      if (encoderChange < -180.0) encoderChange += 360.0;
      if (encoderChange > 180.0) encoderChange -= 360.0;
      
      Serial.println("Step " + String(step) + " (" + String(percentComplete, 1) + "%) - " +
                    "Encoder: " + String(currentPosition, 2) + "° - " +
                    "Change: " + String(encoderChange, 2) + "°");
      
      readingIndex++;
    }
  }
  
  // Calculate final results
  delay(100);
  readAS5600();
  float finalPos = currentPosition;
  
  // Calculate total encoder change (handle wraparound)
  float totalEncoderChange = finalPos - startPos;
  if (totalEncoderChange < -180.0) totalEncoderChange += 360.0;
  if (totalEncoderChange > 180.0) totalEncoderChange -= 360.0;
  
  // Calculate ratio
  float stepsPerEncoderDegree = totalMotorSteps / abs(totalEncoderChange);
  float encoderDegreesPerStep = abs(totalEncoderChange) / totalMotorSteps;
  
  Serial.println("\n=== CALIBRATION RESULTS ===");
  Serial.println("Start position: " + String(startPos, 2) + "°");
  Serial.println("Final position: " + String(finalPos, 2) + "°");
  Serial.println("Total encoder change: " + String(totalEncoderChange, 2) + "°");
  Serial.println("Motor steps taken: " + String(totalMotorSteps));
  Serial.println("Motor steps per encoder degree: " + String(stepsPerEncoderDegree, 2));
  Serial.println("Encoder degrees per motor step: " + String(encoderDegreesPerStep, 4));
  
  if (abs(totalEncoderChange) > 350.0) {
    Serial.println("✓ Calibration looks good - close to 360°");
  } else {
    Serial.println("⚠ Warning: Encoder change less than expected");
    Serial.println("  Check: mechanical coupling, magnet alignment, or gear ratio");
  }
  
  // Suggest correction factor
  float correctionFactor = 360.0 / abs(totalEncoderChange);
  Serial.println("Suggested correction factor: " + String(correctionFactor, 4));
  Serial.println("===============================\n");
}

void moveSteps(int steps, bool clockwise) {
  // Read starting position
  readAS5600();
  float startPos = currentPosition;
  
  Serial.println("Start position: " + String(startPos, 2) + "°");
  
  // Set direction
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  
  // Move the specified number of steps
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
    
    // Show progress every 100 steps
    if ((i + 1) % 100 == 0) {
      readAS5600();
      float change = currentPosition - startPos;
      if (change < -180.0) change += 360.0;
      if (change > 180.0) change -= 360.0;
      
      Serial.println("Step " + String(i + 1) + "/" + String(steps) + 
                    " - Position: " + String(currentPosition, 2) + 
                    "° - Change: " + String(change, 2) + "°");
    }
  }
  
  // Final reading
  delay(50);
  readAS5600();
  float finalPos = currentPosition;
  float totalChange = finalPos - startPos;
  if (totalChange < -180.0) totalChange += 360.0;
  if (totalChange > 180.0) totalChange -= 360.0;
  
  Serial.println("Final position: " + String(finalPos, 2) + "°");
  Serial.println("Total change: " + String(totalChange, 2) + "°");
  Serial.println("Degrees per step: " + String(totalChange / steps, 4) + "°");
  Serial.println();
}

void readAS5600() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    // Read 12-bit angle value
    uint16_t rawAngle = Wire.read() << 8;
    rawAngle |= Wire.read();
    
    // Convert to degrees (12-bit = 4096 counts for 360°)
    currentPosition = (rawAngle * 360.0) / 4096.0;
    
    // Optional: Apply calibration offset if needed
    // currentPosition += CALIBRATION_OFFSET;
    
    // Normalize to 0-360 range
    while (currentPosition >= 360.0) currentPosition -= 360.0;
    while (currentPosition < 0.0) currentPosition += 360.0;
  } else {
    Serial.println("Error reading AS5600 encoder");
  }
}

// Alternative function to read AS5600 with error handling
bool readAS5600Safe() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REG);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println("I2C transmission error: " + String(error));
    return false;
  }
  
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    uint16_t rawAngle = Wire.read() << 8;
    rawAngle |= Wire.read();
    
    // Check for valid reading (AS5600 returns 0xFFFF if magnet not detected)
    if (rawAngle == 0xFFFF) {
      Serial.println("Warning: No magnet detected by AS5600");
      return false;
    }
    
    currentPosition = (rawAngle * 360.0) / 4096.0;
    
    // Normalize to 0-360 range
    while (currentPosition >= 360.0) currentPosition -= 360.0;
    while (currentPosition < 0.0) currentPosition += 360.0;
    
    return true;
  }
  
  return false;
}
// MPU6050_Pedometer_JumpCounter_Complete.ino
// ESP32 + MPU6050 pedometer + jump counter
// Task 1-5: Basic Counting & Logic
// Task 6: Jump Height Calculation (Time of Flight)
// Task 7: Step Length Estimation (Weinberg approach approximation)

#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ============================================================================
// CONFIGURATION
// ============================================================================

// Hardware pins
#define SDA_PIN 21
#define SCL_PIN 22
const uint8_t MPU_ADDR = 0x68;

// MPU6050 registers
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_SMPRT_DIV = 0x19;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_WHO_AM_I = 0x75;

// Sampling configuration
const float SAMPLE_FREQ = 100.0f;
const unsigned long SAMPLE_PERIOD_MS = (unsigned long)(1000.0f / SAMPLE_FREQ);
const float ACCEL_SENSITIVITY = 16384.0f; // LSB/g for ±2g

// Filter constants
const float ALPHA_LP = 0.95f;
const float EMA_ALPHA = 0.03f;

// Adaptive threshold configuration
struct ThresholdConfig {
  bool enabled = true;
  float adaptK = 0.20f;       // Reduced from 0.30
  float minThreshold = 0.13f; // Reduced from 0.20
  float maxThreshold = 1.6f;
  float releaseFactorStep = 0.85f; // Increased from 0.8
};

// Step detection configuration
struct StepConfig {
  unsigned long minIntervalMs = 180;      // Lowered slightly to catch fast walking
  unsigned long suspendAfterJumpMs = 800; // Keeps step counting off while you stabilize after landing
  unsigned long preJumpWindowMs = 750;    // Look back 750ms to delete the "push off" step
  float stepLengthK = 0.45f;              // Constant for step length estimation (Weinberg approximation)
};

// Jump detection configuration
struct JumpConfig {
  float freefallThresh = 0.85f;     // Middle value
  float landingThresh = 1.6f;       // Middle value
  float landingToStepFactor = 1.6f; 
  unsigned long minAirTimeMs = 100; // Set back to 100
  unsigned long maxAirTimeMs = 900;
};

// Cadence tracking
const unsigned long CADENCE_WINDOW_MS = 5000;
const int MAX_STEP_EVENTS = 200;

// Status reporting
const unsigned long STATUS_REPORT_INTERVAL_MS = 1000;

// Error recovery
const unsigned long MPU_RECONNECT_INTERVAL_MS = 5000;
const int MAX_READ_FAILURES = 5;

// ============================================================================
// GLOBAL STATE
// ============================================================================

ThresholdConfig thresholdCfg;
StepConfig stepCfg;
JumpConfig jumpCfg;

// Gravity baseline (calibrated at startup)
struct GravityVector {
  float x = 0, y = 0, z = 0;
} gravity;

// Adaptive threshold state
struct AdaptiveState {
  float peakThreshold = 0.5f;
  float emaMag = 0.0f;
  float emaMagSq = 0.0f;
} adaptive;

// Timing
unsigned long lastSampleMs = 0;
unsigned long lastStepMs = 0;
unsigned long lastJumpDetectMs = 0;
unsigned long stepSuspendedUntil = 0;
unsigned long lastStatusReportMs = 0;

// Counters & Metrics
int stepCount = 0;
int jumpCount = 0;
float totalDistanceMeters = 0.0f; // Task 7
float lastStepLengthMeters = 0.0f;
float lastJumpHeightCm = 0.0f;    // Task 6

// Step timing circular buffer
unsigned long stepTimes[MAX_STEP_EVENTS] = {0};
int stepTimesHead = 0;

// Jump state machine
enum JumpState { WAIT_FOR_AIR, IN_AIR, WAIT_FOR_LAND };
JumpState jumpState = WAIT_FOR_AIR;

// Step detection state
bool waitingForPeak = true;

// Error tracking
int consecutiveReadFailures = 0;
bool mpuConnected = true;
unsigned long lastReconnectAttempt = 0;

// ============================================================================
// I2C COMMUNICATION
// ============================================================================

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  
  uint8_t received = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (received != len) return false;
  
  for (uint8_t i = 0; i < len; i++) {
    if (Wire.available()) {
      buf[i] = Wire.read();
    } else {
      return false;
    }
  }
  return true;
}

// ============================================================================
// MPU6050 INITIALIZATION
// ============================================================================

bool checkMPU6050() {
  uint8_t whoami = 0;
  if (!readRegs(REG_WHO_AM_I, &whoami, 1)) return false;
  return (whoami == 0x68 || whoami == 0x72); // MPU6050 or compatible
}

bool setupMPU6050() {
  // Wake up device
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);
  
  // Configure sample rate (200Hz internal, divide by 5 = 40Hz output)
  if (!writeReg(REG_SMPRT_DIV, 0x04)) return false;
  
  // Configure DLPF (94Hz bandwidth for accel, helps filter noise)
  if (!writeReg(REG_CONFIG, 0x03)) return false;
  
  // Configure accelerometer range (±2g)
  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) return false;
  
  delay(20);
  return true;
}

// ============================================================================
// CALIBRATION
// ============================================================================

bool calibrateGravity() {
  const int warmupSamples = 50;
  const int calibrationSamples = 200;
  
  // Warmup period (discard initial readings)
  for (int i = 0; i < warmupSamples; i++) {
    uint8_t buf[6];
    readRegs(REG_ACCEL_XOUT_H, buf, 6);
    delay(5);
  }
  
  // Actual calibration
  double sumx = 0, sumy = 0, sumz = 0;
  int validSamples = 0;
  
  for (int i = 0; i < calibrationSamples; i++) {
    uint8_t buf[6];
    if (readRegs(REG_ACCEL_XOUT_H, buf, 6)) {
      int16_t ax = (buf[0] << 8) | buf[1];
      int16_t ay = (buf[2] << 8) | buf[3];
      int16_t az = (buf[4] << 8) | buf[5];
      sumx += ax;
      sumy += ay;
      sumz += az;
      validSamples++;
    }
    delay(5);
  }
  
  if (validSamples < calibrationSamples * 0.9) {
    return false; // Too many failed reads
  }
  
  gravity.x = (sumx / validSamples) / ACCEL_SENSITIVITY;
  gravity.y = (sumy / validSamples) / ACCEL_SENSITIVITY;
  gravity.z = (sumz / validSamples) / ACCEL_SENSITIVITY;
  
  return true;
}

// ============================================================================
// STEP TIMING MANAGEMENT
// ============================================================================

void recordStepTime(unsigned long t) {
  stepTimes[stepTimesHead] = t;
  stepTimesHead = (stepTimesHead + 1) % MAX_STEP_EVENTS;
}

int stepsInWindow(unsigned long now, unsigned long windowMs) {
  int count = 0;
  for (int i = 0; i < MAX_STEP_EVENTS; i++) {
    if (stepTimes[i] > 0 && (now - stepTimes[i] <= windowMs)) {
      count++;
    }
  }
  return count;
}

bool removeLastRecordedStepTime(unsigned long windowMs) {
  int latestIndex = -1;
  unsigned long latestTime = 0;
  unsigned long now = millis();
  
  for (int i = 0; i < MAX_STEP_EVENTS; i++) {
    if (stepTimes[i] > latestTime) {
      latestTime = stepTimes[i];
      latestIndex = i;
    }
  }
  
  if (latestIndex >= 0 && (now - latestTime <= windowMs)) {
    stepTimes[latestIndex] = 0;
    return true;
  }
  return false;
}

// ============================================================================
// ADAPTIVE THRESHOLD UPDATE
// ============================================================================

void updateAdaptiveThreshold(float magnitude) {
  if (!thresholdCfg.enabled) return;
  
  adaptive.emaMag = (1 - EMA_ALPHA) * adaptive.emaMag + EMA_ALPHA * magnitude;
  adaptive.emaMagSq = (1 - EMA_ALPHA) * adaptive.emaMagSq + EMA_ALPHA * magnitude * magnitude;
  
  float variance = adaptive.emaMagSq - adaptive.emaMag * adaptive.emaMag;
  variance = max(0.0f, variance);
  float stdDev = sqrt(variance);
  
  float newThreshold = adaptive.emaMag + thresholdCfg.adaptK * stdDev;
  adaptive.peakThreshold = constrain(newThreshold, 
                                     thresholdCfg.minThreshold, 
                                     thresholdCfg.maxThreshold);
}

// ============================================================================
// STEP DETECTION
// ============================================================================

void processStepDetection(float magnitude, unsigned long now) {
  bool stepEnabled = (now >= stepSuspendedUntil) && (jumpState == WAIT_FOR_AIR);
  
  if (!stepEnabled) {
    waitingForPeak = true;
    return;
  }
  
  // Peak detection with hysteresis
  if (magnitude > adaptive.peakThreshold && waitingForPeak) {
    if (now - lastStepMs > stepCfg.minIntervalMs) {
      stepCount++;
      lastStepMs = now;
      recordStepTime(now);
      waitingForPeak = false;

      // --- TASK 7: Step Length Calculation ---
      // Weinberg Estimation approx: Length = K * (PeakAccel)^(1/4)
      // We use magnitude as a proxy for Peak Accel.
      // 0.45 is a generic K factor for metric units (meters).
      float estimatedLength = stepCfg.stepLengthK * pow(magnitude, 0.25);
      totalDistanceMeters += estimatedLength;
      lastStepLengthMeters = estimatedLength;
    }
  }
  
  // Reset for next peak (hysteresis prevents double-counting)
  if (magnitude < (adaptive.peakThreshold * thresholdCfg.releaseFactorStep)) {
    waitingForPeak = true;
  }
}

// ============================================================================
// JUMP DETECTION
// ============================================================================

void processJumpDetection(float magnitude, unsigned long now) {
  switch (jumpState) {
    case WAIT_FOR_AIR:
      if (magnitude < jumpCfg.freefallThresh) {
        jumpState = IN_AIR;
        lastJumpDetectMs = now;
      }
      break;
      
    case IN_AIR:
      if (magnitude >= jumpCfg.freefallThresh) {
        jumpState = WAIT_FOR_AIR; // False alarm (gravity returned too fast)
      } else if (now - lastJumpDetectMs > jumpCfg.minAirTimeMs) {
        jumpState = WAIT_FOR_LAND;
      }
      break;
      
    case WAIT_FOR_LAND:
      // Check if we hit the ground
      if (magnitude > jumpCfg.landingThresh) {
        
        // 1. Valid Jump Detected
        jumpCount++;
        stepSuspendedUntil = now + stepCfg.suspendAfterJumpMs;
        
        // --- TASK 6: Jump Height Calculation ---
        // Flight Time = (Now - Start of Air)
        // Height (cm) = 1/2 * g * (t/2)^2 * 100
        // g = 9.81 m/s^2, t is in seconds. 
        // Simplified constant: 122.625 * t^2 (where t is flight time in seconds)
        unsigned long flightTimeMs = now - lastJumpDetectMs;
        float flightTimeSec = flightTimeMs / 1000.0f;
        lastJumpHeightCm = 122.625f * (flightTimeSec * flightTimeSec);

        // Debug Jump Info
        Serial.printf(" [JUMP] AirTime: %lums  Height: %.1fcm\n", flightTimeMs, lastJumpHeightCm);

        // 2. REMOVE "PUSH-OFF" STEP
        if (now - lastStepMs <= stepCfg.preJumpWindowMs) {
           // Try to remove the last recorded step time
           if (removeLastRecordedStepTime(stepCfg.preJumpWindowMs) && stepCount > 0) {
             stepCount--;
             // Also remove the distance added by that false step
             // We approximate by removing the average stride (Total / Count)
             if (stepCount > 0) {
                totalDistanceMeters -= (totalDistanceMeters / (stepCount + 1)); 
             } else {
                totalDistanceMeters = 0;
             }
             Serial.println(" [Correction] Push-off step removed."); 
           }
           lastStepMs = 0; // Clear this so we don't remove it twice
        }

        jumpState = WAIT_FOR_AIR;
        
      } else if (now - lastJumpDetectMs > jumpCfg.maxAirTimeMs) {
        jumpState = WAIT_FOR_AIR; // Timeout (floating in space?)
      }
      break;
  }
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

void printStatus(unsigned long now) {
  int recentSteps = stepsInWindow(now, CADENCE_WINDOW_MS);
  float cadence = (float)recentSteps / ((float)CADENCE_WINDOW_MS / 60000.0f);
  
  char statusMsg[128];
  snprintf(statusMsg, sizeof(statusMsg),
           "Stps:%d Len:%.2fm Jmps:%d LastH:%.1fcm", // Changed Dist to Len, %.1f to %.2f for better precision
           stepCount, lastStepLengthMeters, jumpCount, lastJumpHeightCm); // Replaced totalDistanceMeters
  
  Serial.println(statusMsg);
  if (SerialBT.hasClient()) {
    SerialBT.println(statusMsg);
  }
}

// ============================================================================
// ERROR RECOVERY
// ============================================================================

bool attemptMPUReconnect() {
  Serial.println("Attempting MPU6050 reconnection...");
  
  if (checkMPU6050() && setupMPU6050() && calibrateGravity()) {
    Serial.println("MPU6050 reconnected successfully");
    consecutiveReadFailures = 0;
    mpuConnected = true;
    return true;
  }
  
  Serial.println("MPU6050 reconnection failed");
  return false;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== MPU6050 Pedometer + Jump Counter ===");
  Serial.println("Initializing...");
  
  // Initialize Bluetooth
  if (!SerialBT.begin("ESP32_MPU6050_Tejas")) {
    Serial.println("Bluetooth initialization failed!");
  } else {
    Serial.println("Bluetooth started: ESP32_MPU6050_Tejas");
  }
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setTimeOut(1000); // 1 second timeout
  
  // Check and initialize MPU6050
  if (!checkMPU6050()) {
    Serial.println("ERROR: MPU6050 not found!");
    mpuConnected = false;
  } else if (!setupMPU6050()) {
    Serial.println("ERROR: MPU6050 setup failed!");
    mpuConnected = false;
  } else if (!calibrateGravity()) {
    Serial.println("ERROR: Calibration failed!");
    mpuConnected = false;
  } else {
    Serial.println("MPU6050 initialized successfully");
    Serial.printf("Gravity baseline: X=%.3f Y=%.3f Z=%.3f\n", 
                  gravity.x, gravity.y, gravity.z);
    mpuConnected = true;
  }
  
  lastSampleMs = millis();
  lastStatusReportMs = lastSampleMs;
  Serial.println("Ready!\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();
  
  // Handle MPU reconnection if disconnected
  if (!mpuConnected) {
    if (now - lastReconnectAttempt >= MPU_RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = now;
      attemptMPUReconnect();
    }
    delay(100);
    return;
  }
  
  // Sample at fixed rate
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSampleMs += SAMPLE_PERIOD_MS;
  
  // Read accelerometer data
  uint8_t buf[6];
  if (!readRegs(REG_ACCEL_XOUT_H, buf, 6)) {
    consecutiveReadFailures++;
    if (consecutiveReadFailures >= MAX_READ_FAILURES) {
      Serial.println("ERROR: Multiple read failures, marking MPU as disconnected");
      mpuConnected = false;
    }
    return;
  }
  consecutiveReadFailures = 0;
  
  // Convert to g-force
  int16_t axRaw = (buf[0] << 8) | buf[1];
  int16_t ayRaw = (buf[2] << 8) | buf[3];
  int16_t azRaw = (buf[4] << 8) | buf[5];
  
  float ax = axRaw / ACCEL_SENSITIVITY;
  float ay = ayRaw / ACCEL_SENSITIVITY;
  float az = azRaw / ACCEL_SENSITIVITY;
  
  // Update gravity estimate (low-pass filter)
  gravity.x = ALPHA_LP * gravity.x + (1 - ALPHA_LP) * ax;
  gravity.y = ALPHA_LP * gravity.y + (1 - ALPHA_LP) * ay;
  gravity.z = ALPHA_LP * gravity.z + (1 - ALPHA_LP) * az;
  
  // High-pass filter: remove gravity to get dynamic acceleration
  float dynX = ax - gravity.x;
  float dynY = ay - gravity.y;
  float dynZ = az - gravity.z;
  float magnitude = sqrt(dynX * dynX + dynY * dynY + dynZ * dynZ);
  
  // Update adaptive threshold
  updateAdaptiveThreshold(magnitude);
  
  // Process step and jump detection
  processStepDetection(magnitude, now);
  processJumpDetection(magnitude, now);
  
  // Periodic status output
  if (now - lastStatusReportMs >= STATUS_REPORT_INTERVAL_MS) {
    lastStatusReportMs = now;
    printStatus(now);
  }
}
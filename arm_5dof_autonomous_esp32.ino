/*
  ============================================================================
  5-DOF Arm — Autonomous Pick-and-Place (No Serial Commands Required)
  Board:    ESP32 (hardware I2C to PCA9685)
  ============================================================================
  Runs a hardcoded sequence of positions automatically when the ESP32 powers
  on — no computer or serial commands needed for operation. Serial output is
  for status/debugging only (optional to view; the arm runs the same way
  whether or not anything is watching the serial monitor).

  ----------------------------------------------------------------------------
  HOW TO SET THIS UP FOR YOUR DEMO
  ----------------------------------------------------------------------------
  1. Use the earlier interactive sketch (with J/S/P/L/R commands) connected
     to a computer to jog the arm and find the exact angles for every step
     of your routine — above each word, down onto it, gripper closed, lifted,
     above the plate target, down, gripper open, lifted, and so on.
  2. Use that sketch's "L" command to print out each saved position.
  3. Copy those numbers into the sequence[][5] array below, in the exact
     order you want them executed. Each row is one step:
         {base, shoulder, elbow, wrist, gripper}
     gripper: 0 = fully open, 180 = fully closed (logical value — mirrored
     into both physical gripper servos automatically).
  4. Update SEQUENCE_LENGTH to match the number of rows you have.
  5. Update angleMin/angleMax and servoMinPulse/servoMaxPulse below to match
     whatever you calibrated on your specific arm.
  6. Upload this sketch, then unplug it from the computer, power it from
     your battery setup, and it will run the whole sequence automatically
     the moment it powers on.

  The example sequence below is a PLACEHOLDER for one word (approach, grab,
  lift, move, place, release, lift, home) — replace every row with your own
  taught positions before the actual demo. Running this with the placeholder
  numbers on your real arm risks a collision, since they weren't measured
  against your physical setup.

  WIRING: unchanged — ESP32 3V3->PCA9685 VCC, GND->GND, GPIO21->SDA,
  GPIO22->SCL, PCA9685 V+ left disconnected, servo power from the separate
  power bus.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
// I2C configuration (ESP32 hardware I2C peripheral, for the PCA9685)
// ---------------------------------------------------------------------------
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_CLOCK_HZ 400000UL

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------------------------------------------------------------------
// PCA9685 channel assignment (6 physical servos)
// ---------------------------------------------------------------------------
const uint8_t NUM_SERVOS = 6;

enum ServoChannel : uint8_t {
  CH_BASE      = 0,
  CH_SHOULDER  = 1,
  CH_ELBOW     = 2,
  CH_WRIST     = 3,
  CH_GRIPPER_L = 4,
  CH_GRIPPER_R = 5
};

// PCA9685 pulse length range per servo, in ticks. CALIBRATE per servo.
const uint16_t servoMinPulse[NUM_SERVOS] = {102, 102, 102, 102, 102, 102};
const uint16_t servoMaxPulse[NUM_SERVOS] = {512, 512, 512, 512, 512, 512};

// Mechanical angle range per channel, in degrees. CALIBRATE to your arm's
// actual safe range of motion.
const uint16_t angleMin[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
const uint16_t angleMax[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};

// Gripper mirroring: one servo is mounted opposite the other.
const bool GRIPPER_L_INVERT = false;
const bool GRIPPER_R_INVERT = true;   // flip if the gripper opens/closes wrong

// ---------------------------------------------------------------------------
// HARDCODED SEQUENCE — replace every row with your own taught positions
// Each row: {base, shoulder, elbow, wrist, gripper}
// ---------------------------------------------------------------------------
const uint8_t SEQUENCE_LENGTH = 9;

const float sequence[SEQUENCE_LENGTH][5] = {
  {90,  90, 90, 90,   0},   // 0: home / start
  {60,  60, 90, 60,   0},   // 1: above word (gripper open)
  {60,  40, 60, 60,   0},   // 2: descend onto word (open)
  {60,  40, 60, 60, 150},   // 3: close gripper around word
  {60,  60, 90, 60, 150},   // 4: lift word clear of table
  {120, 60, 90, 60, 150},   // 5: move above plate target (closed)
  {120, 40, 60, 60, 150},   // 6: descend onto plate target (closed)
  {120, 40, 60, 60,   0},   // 7: open gripper, release word
  {90,  90, 90, 90,   0},   // 8: lift and return home
};

// ---------------------------------------------------------------------------
// Motion tuning
// ---------------------------------------------------------------------------
const float MAX_JOINT_SPEED_DEG_PER_SEC = 60.0f;
const unsigned long STEP_DWELL_MS = 400;   // pause at each step before the next
const unsigned long STARTUP_DELAY_MS = 2000; // pause after power-on before moving

float   targetAngle[NUM_SERVOS];
float   currentAngle[NUM_SERVOS];
float   moveSpeed[NUM_SERVOS];
uint8_t lastWrittenAngle[NUM_SERVOS];

unsigned long lastMotionUpdate = 0;

// ---------------------------------------------------------------------------
// Function prototypes
// ---------------------------------------------------------------------------
float    fmap(float x, float inMin, float inMax, float outMin, float outMax);
uint16_t angleToPulse(uint8_t servoIndex, float angleDeg);
void     writeServoAngle(uint8_t servoIndex, float angleDeg);
void     applyPoseToTargets(const float pose[5]);
void     updateMotion();
bool     isMoveComplete();
void     runSequence();
void     goToPoseAndWait(const float pose[5], uint8_t stepNumber);

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Command every channel to a neutral starting position before anything
  // else happens, so no servo is left uncommanded at power-on.
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    currentAngle[ch] = 90.0f;
    targetAngle[ch] = 90.0f;
    lastWrittenAngle[ch] = 90;
    writeServoAngle(ch, 90.0f);
  }
  lastMotionUpdate = millis();

  Serial.print("Arm powered on. Starting sequence in ");
  Serial.print(STARTUP_DELAY_MS / 1000);
  Serial.println(" seconds...");
  delay(STARTUP_DELAY_MS);

  runSequence();

  Serial.println("Sequence complete. Arm holding final position.");
}

// ---------------------------------------------------------------------------
void loop() {
  // The sequence already ran once in setup(). Keep the motion engine
  // ticking so the arm holds its final commanded position — nothing else
  // to do without any serial input to react to.
  updateMotion();
  delay(15);
}

// ---------------------------------------------------------------------------
// Runs through every step in the hardcoded sequence, in order, waiting for
// each move to finish (plus a short dwell) before starting the next.
void runSequence() {
  for (uint8_t i = 0; i < SEQUENCE_LENGTH; i++) {
    goToPoseAndWait(sequence[i], i);
  }
}

void goToPoseAndWait(const float pose[5], uint8_t stepNumber) {
  Serial.print("Step "); Serial.print(stepNumber); Serial.println(": moving...");

  applyPoseToTargets(pose);

  while (!isMoveComplete()) {
    updateMotion();
    delay(15);
  }

  delay(STEP_DWELL_MS);
  Serial.print("Step "); Serial.print(stepNumber); Serial.println(": done.");
}

// ---------------------------------------------------------------------------
// Converts a 5-value logical pose into targets for all 6 physical channels,
// and computes a per-channel speed so every servo starts and finishes the
// move at the same time (synchronized arrival).
void applyPoseToTargets(const float pose[5]) {
  float fullTarget[NUM_SERVOS];
  fullTarget[CH_BASE]     = constrain(pose[0], (float)angleMin[CH_BASE],     (float)angleMax[CH_BASE]);
  fullTarget[CH_SHOULDER] = constrain(pose[1], (float)angleMin[CH_SHOULDER], (float)angleMax[CH_SHOULDER]);
  fullTarget[CH_ELBOW]    = constrain(pose[2], (float)angleMin[CH_ELBOW],    (float)angleMax[CH_ELBOW]);
  fullTarget[CH_WRIST]    = constrain(pose[3], (float)angleMin[CH_WRIST],    (float)angleMax[CH_WRIST]);

  float logicalGripper = constrain(pose[4], 0.0f, 180.0f);
  float fraction  = fmap(logicalGripper, 0.0f, 180.0f, 0.0f, 1.0f);
  float fractionL = GRIPPER_L_INVERT ? (1.0f - fraction) : fraction;
  float fractionR = GRIPPER_R_INVERT ? (1.0f - fraction) : fraction;
  fullTarget[CH_GRIPPER_L] = fmap(fractionL, 0.0f, 1.0f, angleMin[CH_GRIPPER_L], angleMax[CH_GRIPPER_L]);
  fullTarget[CH_GRIPPER_R] = fmap(fractionR, 0.0f, 1.0f, angleMin[CH_GRIPPER_R], angleMax[CH_GRIPPER_R]);

  float maxDistance = 0.0f;
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    float dist = fabs(fullTarget[ch] - currentAngle[ch]);
    if (dist > maxDistance) maxDistance = dist;
  }

  float moveDuration = maxDistance / MAX_JOINT_SPEED_DEG_PER_SEC;
  if (moveDuration < 0.02f) moveDuration = 0.02f;

  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    float dist = fabs(fullTarget[ch] - currentAngle[ch]);
    moveSpeed[ch] = dist / moveDuration;
    targetAngle[ch] = fullTarget[ch];
  }
}

// ---------------------------------------------------------------------------
void updateMotion() {
  unsigned long now = millis();
  float deltaTime = (now - lastMotionUpdate) / 1000.0f;
  lastMotionUpdate = now;

  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    float maxStep = moveSpeed[ch] * deltaTime;

    if (currentAngle[ch] < targetAngle[ch]) {
      currentAngle[ch] = min(currentAngle[ch] + maxStep, targetAngle[ch]);
    } else if (currentAngle[ch] > targetAngle[ch]) {
      currentAngle[ch] = max(currentAngle[ch] - maxStep, targetAngle[ch]);
    }

    uint8_t angleToWrite = (uint8_t)round(currentAngle[ch]);
    if (angleToWrite != lastWrittenAngle[ch]) {
      writeServoAngle(ch, currentAngle[ch]);
      lastWrittenAngle[ch] = angleToWrite;
    }
  }
}

bool isMoveComplete() {
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    if (fabs(currentAngle[ch] - targetAngle[ch]) > 0.5f) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
float fmap(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

uint16_t angleToPulse(uint8_t servoIndex, float angleDeg) {
  angleDeg = constrain(angleDeg, (float)angleMin[servoIndex], (float)angleMax[servoIndex]);
  float pulse = fmap(angleDeg, 0, 180, servoMinPulse[servoIndex], servoMaxPulse[servoIndex]);
  return (uint16_t)round(pulse);
}

void writeServoAngle(uint8_t servoIndex, float angleDeg) {
  uint16_t pulse = angleToPulse(servoIndex, angleDeg);
  pwm.setPWM(servoIndex, 0, pulse);
}

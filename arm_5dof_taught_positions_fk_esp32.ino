/*
  ============================================================================
  5-DOF Arm — Taught Positions + Forward Kinematics Diagnostic
  Board:    ESP32 (hardware I2C to PCA9685, USB serial for commands)
  ============================================================================
  Builds on the taught-positions system (jog/save/play/list/run) and adds a
  FORWARD KINEMATICS diagnostic: given a set of joint angles, compute where
  the gripper physically ends up in (x, y, z), so you can numerically verify
  things like "is this taught pickup position actually past the 25cm plate
  radius" instead of only checking by eye.

  This is diagnostic only — it does NOT change how the arm is controlled.
  You still teach positions by jogging (as before); FK just reports where a
  given set of angles physically lands.

  COMMANDS (send over Serial, 115200 baud, newline-terminated)
  ------------------------------------------------------------------------
  J,base,shoulder,elbow,wrist,gripper     Jog directly to a position
  S,slot                                  Save the CURRENT position into slot
  S,slot,base,shoulder,elbow,wrist,grip   Define a slot directly (import)
  P,slot                                  Move to a saved position
  L                                       List saved positions, now WITH
                                           their FK-computed x/y/z and reach
  R,slot1,slot2,...                       Run a sequence
  C,slot                                  Clear a saved position
  H                                       Move to the home position
  F                                       FK for the CURRENT live position
  F,slot                                  FK for a specific saved position

  ----------------------------------------------------------------------------
  CALIBRATING FORWARD KINEMATICS (required before the numbers mean anything)
  ----------------------------------------------------------------------------
  FK needs two things from you that the code cannot know on its own:

  1. LINK LENGTHS — measure your arm with a ruler, in the same unit
     throughout (cm is used here to match the 25cm plate radius):
       LINK_L1      shoulder pivot -> elbow pivot
       LINK_L2      elbow pivot -> wrist pivot
       LINK_L3      wrist pivot -> gripper fingertip (the actual grab point)
       BASE_HEIGHT  table/ground surface -> shoulder pivot height

  2. PER-JOINT ANGLE CALIBRATION — a servo's commanded angle (e.g. 90) does
     not necessarily mean the same thing physically as "arm horizontal" or
     "arm straight" unless you tell the code the relationship. For each
     joint:
       - Use the J command to move that joint until the link is at a KNOWN
         physical angle:
           BASE:     pointing along whatever direction you call 0 degrees
           SHOULDER: upper arm horizontal
           ELBOW:    forearm in a straight line with the upper arm
           WRIST:    whatever you define as the wrist's 0-degree reference
       - Whatever servo angle you commanded to get there is that joint's
         ZERO_OFFSET below.
       - DIRECTION is +1 or -1: increase the servo angle a little more and
         check whether the physical angle increases (+1) or decreases (-1)
         in the same rotational sense used above.

  Do this once per joint, fill in the constants below, then re-upload.
  Compare the F command's output against a tape measure on a taught position
  to sanity-check your calibration before trusting the numbers.

  WIRING: unchanged — ESP32 3V3->PCA9685 VCC, GND->GND, GPIO21->SDA,
  GPIO22->SCL, PCA9685 V+ left disconnected, servo power from the separate
  power bus.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

Preferences prefs;

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

const uint16_t servoMinPulse[NUM_SERVOS] = {102, 102, 102, 102, 102, 102};
const uint16_t servoMaxPulse[NUM_SERVOS] = {512, 512, 512, 512, 512, 512};

const uint16_t angleMin[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
const uint16_t angleMax[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};

const bool GRIPPER_L_INVERT = false;
const bool GRIPPER_R_INVERT = true;

const float HOME_POSE[5] = {90, 90, 90, 90, 0};

// ---------------------------------------------------------------------------
// Forward kinematics — MEASURE AND CALIBRATE THESE FOR YOUR ARM
// ---------------------------------------------------------------------------
const float LINK_L1 = 15.0f;      // shoulder pivot -> elbow pivot
const float LINK_L2 = 15.0f;      // elbow pivot -> wrist pivot
const float LINK_L3 = 8.0f;       // wrist pivot -> gripper fingertip
const float BASE_HEIGHT = 10.0f;  // table/ground -> shoulder pivot

const float BASE_ZERO_OFFSET     = 90.0f;  // servo angle at your 0-degree reference direction
const float BASE_DIRECTION       = 1.0f;

const float SHOULDER_ZERO_OFFSET = 90.0f;  // servo angle when upper arm is horizontal
const float SHOULDER_DIRECTION   = 1.0f;

const float ELBOW_ZERO_OFFSET    = 90.0f;  // servo angle when forearm is in line with upper arm
const float ELBOW_DIRECTION      = 1.0f;

const float WRIST_ZERO_OFFSET    = 90.0f;  // servo angle at your wrist 0-degree reference
const float WRIST_DIRECTION      = 1.0f;

struct Position3D {
  float x, y, z, reach;
};

// ---------------------------------------------------------------------------
// Synchronized multi-joint motion
// ---------------------------------------------------------------------------
const float MAX_JOINT_SPEED_DEG_PER_SEC = 60.0f;

float   targetAngle[NUM_SERVOS];
float   currentAngle[NUM_SERVOS];
float   moveSpeed[NUM_SERVOS];
uint8_t lastWrittenAngle[NUM_SERVOS];

unsigned long lastMotionUpdate = 0;

// ---------------------------------------------------------------------------
// Taught position storage (persisted to flash)
// ---------------------------------------------------------------------------
const uint8_t NUM_POSES = 20;
float poseData[NUM_POSES][5];
bool  poseUsed[NUM_POSES];

const unsigned long SEQUENCE_DWELL_MS = 400;

// ---------------------------------------------------------------------------
// Function prototypes
// ---------------------------------------------------------------------------
float       fmap(float x, float inMin, float inMax, float outMin, float outMax);
uint16_t    angleToPulse(uint8_t servoIndex, float angleDeg);
void        writeServoAngle(uint8_t servoIndex, float angleDeg);
void        applyPoseToTargets(const float pose[5]);
void        updateMotion();
bool        isMoveComplete();
void        loadPosesFromNVS();
void        savePosesToNVS();
void        readSerialCommand();
void        handleJog(const String &line);
void        handleSave(const String &line);
void        handlePlay(const String &line);
void        handleList();
void        handleRun(const String &line);
void        handleClear(const String &line);
void        handleFK(const String &line);
void        moveHome();
Position3D  computeFK(float baseServoAngle, float shoulderServoAngle, float elbowServoAngle, float wristServoAngle);

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  loadPosesFromNVS();

  applyPoseToTargets(HOME_POSE);
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    currentAngle[ch] = targetAngle[ch];
    lastWrittenAngle[ch] = (uint8_t)round(currentAngle[ch]);
    writeServoAngle(ch, currentAngle[ch]);
  }

  lastMotionUpdate = millis();

  Serial.println("Arm ready.");
  Serial.println("Commands:");
  Serial.println("  J,base,shoulder,elbow,wrist,gripper");
  Serial.println("  S,slot  |  S,slot,b,s,e,w,g");
  Serial.println("  P,slot  |  L  |  R,slot1,slot2,...  |  C,slot  |  H");
  Serial.println("  F       (FK for current position)");
  Serial.println("  F,slot  (FK for a saved position)");
}

// ---------------------------------------------------------------------------
void loop() {
  readSerialCommand();
  updateMotion();
  delay(15);
}

// ---------------------------------------------------------------------------
void readSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = toupper(line.charAt(0));

  switch (cmd) {
    case 'J': handleJog(line);   break;
    case 'S': handleSave(line);  break;
    case 'P': handlePlay(line);  break;
    case 'L': handleList();      break;
    case 'R': handleRun(line);   break;
    case 'C': handleClear(line); break;
    case 'H': moveHome();        break;
    case 'F': handleFK(line);    break;
    default:
      Serial.println("ERR: unknown command. Use J, S, P, L, R, C, H, or F");
  }
}

// ---------------------------------------------------------------------------
void handleJog(const String &line) {
  int b, s, e, w, g;
  int n = sscanf(line.c_str(), "J,%d,%d,%d,%d,%d", &b, &s, &e, &w, &g);
  if (n != 5) {
    Serial.println("ERR: J,base,shoulder,elbow,wrist,gripper");
    return;
  }
  float pose[5] = {(float)b, (float)s, (float)e, (float)w, (float)g};
  applyPoseToTargets(pose);
  Serial.println("OK jog");
}

// ---------------------------------------------------------------------------
void handleSave(const String &line) {
  int n, b, s, e, w, g;

  int matched = sscanf(line.c_str(), "S,%d,%d,%d,%d,%d,%d", &n, &b, &s, &e, &w, &g);
  if (matched == 6) {
    if (n < 0 || n >= NUM_POSES) { Serial.println("ERR: slot out of range"); return; }
    poseData[n][0] = b; poseData[n][1] = s; poseData[n][2] = e;
    poseData[n][3] = w; poseData[n][4] = g;
    poseUsed[n] = true;
    savePosesToNVS();
    Serial.print("OK defined slot "); Serial.println(n);
    return;
  }

  matched = sscanf(line.c_str(), "S,%d", &n);
  if (matched != 1) {
    Serial.println("ERR: S,slot  or  S,slot,base,shoulder,elbow,wrist,gripper");
    return;
  }
  if (n < 0 || n >= NUM_POSES) { Serial.println("ERR: slot out of range"); return; }

  poseData[n][0] = currentAngle[CH_BASE];
  poseData[n][1] = currentAngle[CH_SHOULDER];
  poseData[n][2] = currentAngle[CH_ELBOW];
  poseData[n][3] = currentAngle[CH_WRIST];

  float fractionL = fmap(currentAngle[CH_GRIPPER_L], angleMin[CH_GRIPPER_L], angleMax[CH_GRIPPER_L], 0.0f, 1.0f);
  float fraction = GRIPPER_L_INVERT ? (1.0f - fractionL) : fractionL;
  poseData[n][4] = fmap(fraction, 0.0f, 1.0f, 0.0f, 180.0f);

  poseUsed[n] = true;
  savePosesToNVS();

  Serial.print("OK saved slot "); Serial.println(n);
}

// ---------------------------------------------------------------------------
void handlePlay(const String &line) {
  int n;
  if (sscanf(line.c_str(), "P,%d", &n) != 1) {
    Serial.println("ERR: P,slot");
    return;
  }
  if (n < 0 || n >= NUM_POSES || !poseUsed[n]) {
    Serial.println("ERR: slot empty or out of range");
    return;
  }
  applyPoseToTargets(poseData[n]);
  Serial.print("OK playing slot "); Serial.println(n);
}

// ---------------------------------------------------------------------------
void handleList() {
  Serial.println("Saved positions (S,slot,base,shoulder,elbow,wrist,gripper) with FK reach:");
  for (uint8_t i = 0; i < NUM_POSES; i++) {
    if (poseUsed[i]) {
      Serial.print("S,"); Serial.print(i); Serial.print(",");
      Serial.print(poseData[i][0], 0); Serial.print(",");
      Serial.print(poseData[i][1], 0); Serial.print(",");
      Serial.print(poseData[i][2], 0); Serial.print(",");
      Serial.print(poseData[i][3], 0); Serial.print(",");
      Serial.print(poseData[i][4], 0);

      Position3D pos = computeFK(poseData[i][0], poseData[i][1], poseData[i][2], poseData[i][3]);
      Serial.print("   FK x:"); Serial.print(pos.x, 1);
      Serial.print(" y:");      Serial.print(pos.y, 1);
      Serial.print(" z:");      Serial.print(pos.z, 1);
      Serial.print(" reach:");  Serial.println(pos.reach, 1);
    }
  }
  Serial.println("(copy the S,... portion of each line to re-import; FK values are diagnostic only)");
}

// ---------------------------------------------------------------------------
void handleClear(const String &line) {
  int n;
  if (sscanf(line.c_str(), "C,%d", &n) != 1) {
    Serial.println("ERR: C,slot");
    return;
  }
  if (n < 0 || n >= NUM_POSES) { Serial.println("ERR: slot out of range"); return; }
  poseUsed[n] = false;
  savePosesToNVS();
  Serial.print("OK cleared slot "); Serial.println(n);
}

// ---------------------------------------------------------------------------
void handleRun(const String &line) {
  const uint8_t MAX_SEQ = 20;
  int seq[MAX_SEQ];
  uint8_t count = 0;

  int firstComma = line.indexOf(',');
  if (firstComma == -1) { Serial.println("ERR: R,slot1,slot2,..."); return; }
  String rest = line.substring(firstComma + 1);

  int pos = 0;
  while (pos < (int)rest.length() && count < MAX_SEQ) {
    int commaIdx = rest.indexOf(',', pos);
    String token = (commaIdx == -1) ? rest.substring(pos) : rest.substring(pos, commaIdx);
    token.trim();
    if (token.length() > 0) {
      seq[count++] = token.toInt();
    }
    if (commaIdx == -1) break;
    pos = commaIdx + 1;
  }

  if (count == 0) { Serial.println("ERR: R,slot1,slot2,..."); return; }

  Serial.print("OK running sequence of "); Serial.print(count); Serial.println(" positions");

  for (uint8_t i = 0; i < count; i++) {
    int n = seq[i];
    if (n < 0 || n >= NUM_POSES || !poseUsed[n]) {
      Serial.print("ERR: slot "); Serial.print(n); Serial.println(" empty or out of range, stopping sequence");
      return;
    }

    Serial.print("  -> slot "); Serial.println(n);
    applyPoseToTargets(poseData[n]);

    while (!isMoveComplete()) {
      updateMotion();
      delay(15);
    }
    delay(SEQUENCE_DWELL_MS);
  }

  Serial.println("OK sequence complete");
}

// ---------------------------------------------------------------------------
void moveHome() {
  applyPoseToTargets(HOME_POSE);
  Serial.println("OK moving home");
}

// ---------------------------------------------------------------------------
// Forward kinematics: given commanded servo angles for base, shoulder,
// elbow, and wrist, compute where the gripper physically ends up.
// Diagnostic only -- does not affect motion control.
void handleFK(const String &line) {
  int n;
  float b, s, e, w;

  if (sscanf(line.c_str(), "F,%d", &n) == 1) {
    if (n < 0 || n >= NUM_POSES || !poseUsed[n]) {
      Serial.println("ERR: slot empty or out of range");
      return;
    }
    b = poseData[n][0]; s = poseData[n][1]; e = poseData[n][2]; w = poseData[n][3];
  } else {
    b = currentAngle[CH_BASE];
    s = currentAngle[CH_SHOULDER];
    e = currentAngle[CH_ELBOW];
    w = currentAngle[CH_WRIST];
  }

  Position3D pos = computeFK(b, s, e, w);

  Serial.print("FK -> x:"); Serial.print(pos.x, 1);
  Serial.print(" y:");      Serial.print(pos.y, 1);
  Serial.print(" z:");      Serial.print(pos.z, 1);
  Serial.print(" reach:");  Serial.println(pos.reach, 1);
}

// Standard planar-serial-chain forward kinematics on a rotating base.
// Converts commanded servo angles into physical joint angles using each
// joint's ZERO_OFFSET/DIRECTION calibration, then chains the links.
Position3D computeFK(float baseServoAngle, float shoulderServoAngle, float elbowServoAngle, float wristServoAngle) {
  float theta0 = radians(BASE_DIRECTION * (baseServoAngle - BASE_ZERO_OFFSET));
  float alpha1 = radians(SHOULDER_DIRECTION * (shoulderServoAngle - SHOULDER_ZERO_OFFSET));
  float elbowRel = radians(ELBOW_DIRECTION * (elbowServoAngle - ELBOW_ZERO_OFFSET));
  float wristRel = radians(WRIST_DIRECTION * (wristServoAngle - WRIST_ZERO_OFFSET));

  float alpha2 = alpha1 + elbowRel;
  float alpha3 = alpha2 + wristRel;

  float r = LINK_L1 * cos(alpha1) + LINK_L2 * cos(alpha2) + LINK_L3 * cos(alpha3);
  float zLocal = LINK_L1 * sin(alpha1) + LINK_L2 * sin(alpha2) + LINK_L3 * sin(alpha3);

  Position3D pos;
  pos.x = r * cos(theta0);
  pos.y = r * sin(theta0);
  pos.z = BASE_HEIGHT + zLocal;
  pos.reach = sqrt(pos.x * pos.x + pos.y * pos.y);
  return pos;
}

// ---------------------------------------------------------------------------
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
void loadPosesFromNVS() {
  prefs.begin("armposes", true);
  size_t got = prefs.getBytes("poses", poseData, sizeof(poseData));
  size_t gotUsed = prefs.getBytes("used", poseUsed, sizeof(poseUsed));
  prefs.end();

  if (got != sizeof(poseData) || gotUsed != sizeof(poseUsed)) {
    for (uint8_t i = 0; i < NUM_POSES; i++) poseUsed[i] = false;
    Serial.println("No saved positions found — starting fresh.");
  } else {
    Serial.println("Loaded saved positions from flash.");
  }
}

void savePosesToNVS() {
  prefs.begin("armposes", false);
  prefs.putBytes("poses", poseData, sizeof(poseData));
  prefs.putBytes("used", poseUsed, sizeof(poseUsed));
  prefs.end();
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

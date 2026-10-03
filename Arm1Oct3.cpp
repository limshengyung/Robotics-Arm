/*
  ============================================================================
  6-Servo Arm 1 - driven by the base controller (2-bit code + DONE wire)
  Board: ESP32

  Base -> Arm 1 wiring:
    Base GPIO12 (COM_PIN_R1_1, bit 1) -> Arm GPIO2
    Base GPIO13 (COM_PIN_R1_2, bit 0) -> Arm GPIO4
    Arm GPIO17 (DONE)                 -> Base GPIO18 (DONE_PIN_1)
    GND of both boards connected together

  Codes:
    00 HOME      -> SEQUENCE[0]
    01 PICK UP   -> SEQUENCE[1..3]
    10 PUT DOWN  -> SEQUENCE[4]
    11 BACK HOME -> SEQUENCE[5..7]
  Serial monitor still works for manual testing (it cancels the running segment).
  ============================================================================
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <math.h>

const uint8_t NUM_SERVOS = 6;

// ===========================================================================
//                         >>> SEQUENCE SETTINGS <<<
// ===========================================================================
const float KEEP = -1.0f;   // Use KEEP to leave a joint where it is

struct SeqStep {
  float       deg[NUM_SERVOS];  // Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper
  uint32_t    holdMs;           // pause after the step finishes, before the next one
  uint8_t     speedPct;         // Speed of the move INTO this state (10-100 %)
  const char* label;
};

const SeqStep SEQUENCE[] = {
  //   Base  Shldr  Elbow  WPitch WRoll  Grip     hold ms  speed%  label
  { {114, 0, 0, 105, 12, 20},    1000,   100,   "State 1: Home (gripper open)" },
  { {114, 0, 0, 105, 12, 150},    300,   100,   "State 2: Above object"        },
  { {114, 80, 62, 105, 12, 150},  300,    50,   "State 3: Lower to object"     },
  { {164, 80, 62, 105, 12, 150},  500,   100,   "State 4: Close gripper"       },
  { {164, 25, 24, 105, 12, 150},  300,    60,   "State 5: Lift"                },
  { {164, 25, 24, 105, 12, 20},   300,   100,   "State 6: Move to drop point"  },
  { {164, 80, 62, 105, 12, 20},   500,   100,   "State 7: Release"             },
  { {114, 0, 0, 101, 12, 20},    1000,   100,   "State 8: Back to Home"        }
};
const uint8_t SEQ_COUNT = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);
static_assert(sizeof(SEQUENCE) / sizeof(SEQUENCE[0]) >= 8, "SEGMENTS below need 8 states");

const int DEFAULT_RATE_PCT = 100;    // Global playback speed (10-100 %)
// ===========================================================================

// ===========================================================================
//                    >>> LINK TO BASE CONTROLLER <<<
// ===========================================================================
const uint8_t       CODE_PIN_HI    = 2;     // from base COM_PIN_R1_1 (bit 1)
const uint8_t       CODE_PIN_LO    = 4;     // from base COM_PIN_R1_2 (bit 0)
const uint8_t       DONE_OUT       = 17;    // to base DONE_PIN_1
const unsigned long CODE_STABLE_MS = 30;    // code must be steady this long

struct Segment { uint8_t first, last; };    // inclusive SEQUENCE[] indexes
const Segment SEGMENTS[4] = {
  {0, 0},   // 00 HOME
  {1, 3},   // 01 PICK UP
  {4, 4},   // 10 PUT DOWN
  {5, 7}    // 11 BACK HOME
};
const char* CODE_NAME[4] = {"HOME", "PICK UP", "PUT DOWN", "BACK HOME"};
// ===========================================================================

enum EasingType : uint8_t {
  EASE_SINE_IN_OUT    = 0,
  EASE_QUAD_IN_OUT    = 1,
  EASE_CUBIC_IN_OUT   = 2,
  EASE_QUINTIC_IN_OUT = 3,
  EASE_SCURVE         = 4
};

// ---------------------------------------------------------------------------
// Per-joint configuration
// ---------------------------------------------------------------------------
struct JointConfig {
  const char* name;
  bool        enabled;
  uint8_t     pin;
  int         freqHz;
  int         minPulseUs;
  int         maxPulseUs;
  float       minDeg;
  float       maxDeg;
  float       homeDeg;
  float       maxSpeed;   // deg/s
  float       maxAccel;   // deg/s^2
};

const JointConfig JOINTS[NUM_SERVOS] = {
  // name                        en    pin freq minUs maxUs  min    max    home   speed   accel
  { "Base",                      true, 13, 250, 500, 2500,  0.0f, 180.0f, 90.0f,  60.0f, 80.0f },
  { "Shoulder (35kg TD-8135MG)",  true, 14, 250, 500, 2500,  0.0f, 180.0f, 80.0f,  40.0f, 80.0f },
  { "Elbow (25kg TD-8125MG)",     true, 27, 250, 500, 2500,  0.0f, 180.0f, 90.0f,  50.0f, 80.0f },
  { "Wrist Pitch",                true, 26,  50, 500, 2500,  0.0f, 180.0f, 90.0f,  90.0f, 80.0f },
  { "Wrist Roll",                 true, 25,  50, 500, 2500,  0.0f, 180.0f, 90.0f,  90.0f, 80.0f },
  { "Gripper",                    true, 33,  50, 500, 2500,  0.0f, 180.0f, 90.0f, 120.0f, 100.0f }
};

const uint8_t STARTUP_ORDER[NUM_SERVOS] = { 1, 2, 0, 3, 4, 5 };

const unsigned long UPDATE_INTERVAL_US = 4000;
const unsigned long ATTACH_STAGGER_MS  = 250;
const float         SAVE_THRESHOLD_DEG = 0.5f;

EasingType activeEasing = EASE_SCURVE;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
struct JointState {
  float         startDeg;
  float         currentDeg;
  float         targetDeg;
  float         homeDeg;
  float         savedPosDeg;
  float         speed;
  float         accel;
  EasingType    easing;
  unsigned long moveStartUs;
  float         durationSec;
  float         distance;
  float         peakVel;
  float         rampSec;
  float         cruiseSec;
  int           lastUs;
  bool          moving;
  bool          attached;
};

Servo       armServo[NUM_SERVOS];
JointState  js[NUM_SERVOS];
Preferences prefs;

uint8_t       seqIndex       = 0;
int           playRatePct    = DEFAULT_RATE_PCT;

bool          parkedFlag   = false;
bool          parkPending  = false;
int           jogStepDeg   = 5;
unsigned long lastUpdateUs = 0;

// Link state
int8_t        activeCode = -1;       // code currently being executed (-1 = none yet)
bool          segActive  = false;    // a segment is running
bool          holding    = false;    // waiting out holdMs of the finished step
uint8_t       segIdx     = 0;        // SEQUENCE[] index being executed
uint8_t       candCode   = 255;      // code seen on the pins, not yet confirmed
unsigned long candSince  = 0;
unsigned long holdUntil  = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
int degToUs(uint8_t ch, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  const JointConfig& j = JOINTS[ch];
  return (int)roundf(j.minPulseUs + (deg / 180.0f) * (float)(j.maxPulseUs - j.minPulseUs));
}

float clampToJoint(uint8_t ch, float deg) {
  return constrain(deg, JOINTS[ch].minDeg, JOINTS[ch].maxDeg);
}

void writeJointUs(uint8_t ch, int us) {
  if (!js[ch].attached) return;
  if (us != js[ch].lastUs) {
    armServo[ch].writeMicroseconds(us);
    js[ch].lastUs = us;
  }
}

bool anyMoving() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (js[i].moving) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Flash storage (home, last position, parked flag)
// ---------------------------------------------------------------------------
void keyFor(char* buf, size_t len, char prefix, uint8_t ch) {
  snprintf(buf, len, "%c%u", prefix, (unsigned)ch);
}

void saveHome(uint8_t ch) {
  char key[6];
  keyFor(key, sizeof(key), 'h', ch);
  prefs.putFloat(key, js[ch].homeDeg);
}

void savePosition(uint8_t ch, bool forceWrite) {
  if (!forceWrite && fabsf(js[ch].currentDeg - js[ch].savedPosDeg) < SAVE_THRESHOLD_DEG) return;
  char key[6];
  keyFor(key, sizeof(key), 'p', ch);
  prefs.putFloat(key, js[ch].currentDeg);
  js[ch].savedPosDeg = js[ch].currentDeg;
}

void saveAllPositions() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (js[i].attached) savePosition(i, true);
  }
}

void setParked(bool value) {
  if (parkedFlag == value) return;
  parkedFlag = value;
  prefs.putBool("parked", value);
}

// ---------------------------------------------------------------------------
// Easing curves
// ---------------------------------------------------------------------------
float evaluateEasing(float t, EasingType type) {
  t = constrain(t, 0.0f, 1.0f);
  switch (type) {
    case EASE_SINE_IN_OUT:
      return -0.5f * (cosf(PI * t) - 1.0f);
    case EASE_QUAD_IN_OUT:
      return (t < 0.5f) ? (2.0f * t * t)
                        : (1.0f - powf(-2.0f * t + 2.0f, 2.0f) * 0.5f);
    case EASE_CUBIC_IN_OUT:
      return (t < 0.5f) ? (4.0f * t * t * t)
                        : (1.0f - powf(-2.0f * t + 2.0f, 3.0f) * 0.5f);
    case EASE_QUINTIC_IN_OUT:
    default:
      return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
  }
}

void getShapeFactors(EasingType type, float& velFactor, float& accelFactor) {
  switch (type) {
    case EASE_SINE_IN_OUT:  velFactor = 1.5708f; accelFactor = 4.9348f; break;
    case EASE_QUAD_IN_OUT:  velFactor = 2.0f;    accelFactor = 4.0f;    break;
    case EASE_CUBIC_IN_OUT: velFactor = 3.0f;    accelFactor = 12.0f;   break;
    case EASE_QUINTIC_IN_OUT:
    default:                velFactor = 1.875f;  accelFactor = 5.7735f; break;
  }
}

const char* getEasingName(EasingType type) {
  switch (type) {
    case EASE_SINE_IN_OUT:    return "Sine In-Out";
    case EASE_QUAD_IN_OUT:    return "Quadratic In-Out";
    case EASE_CUBIC_IN_OUT:   return "Cubic In-Out";
    case EASE_QUINTIC_IN_OUT: return "Quintic Minimum-Jerk";
    case EASE_SCURVE:         return "S-Curve Trapezoid";
    default:                  return "Unknown";
  }
}

float rampDistance(float v, float Ta, float u) {
  return v * Ta * 0.5f * (u - sinf(PI * u) / PI);
}

float scurvePosition(const JointState& s, float t) {
  const float Ta = s.rampSec;
  const float Tc = s.cruiseSec;
  const float v  = s.peakVel;
  const float T  = 2.0f * Ta + Tc;
  const float dA = v * Ta * 0.5f;

  if (t <= 0.0f)   return 0.0f;
  if (t >= T)      return s.distance;
  if (t < Ta)      return rampDistance(v, Ta, t / Ta);
  if (t < Ta + Tc) return dA + v * (t - Ta);
  return s.distance - rampDistance(v, Ta, (T - t) / Ta);
}

void planSCurve(JointState& s, float vMax, float aMax) {
  const float rampFactor = PI * 0.5f;
  float v  = vMax;
  float Ta = rampFactor * v / aMax;
  float dA = v * Ta * 0.5f;

  if (2.0f * dA > s.distance) {
    v  = sqrtf(s.distance * aMax / rampFactor);
    Ta = rampFactor * v / aMax;
    dA = v * Ta * 0.5f;
  }

  s.peakVel     = v;
  s.rampSec     = Ta;
  s.cruiseSec   = fmaxf(0.0f, (s.distance - 2.0f * dA) / v);
  s.durationSec = 2.0f * Ta + s.cruiseSec;
}

float moveDurationSec(EasingType type, float dist, float vMax, float aMax) {
  if (dist <= 0.05f) return 0.0f;
  if (type == EASE_SCURVE) {
    const float rampFactor = PI * 0.5f;
    float v  = vMax;
    float Ta = rampFactor * v / aMax;
    float dA = v * Ta * 0.5f;
    if (2.0f * dA > dist) {
      v  = sqrtf(dist * aMax / rampFactor);
      Ta = rampFactor * v / aMax;
      dA = v * Ta * 0.5f;
    }
    return 2.0f * Ta + fmaxf(0.0f, (dist - 2.0f * dA) / v);
  }
  float velFactor, accelFactor;
  getShapeFactors(type, velFactor, accelFactor);
  return fmaxf((velFactor * dist) / vMax, sqrtf((accelFactor * dist) / aMax));
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------
bool jointUsable(uint8_t ch) {
  if (!JOINTS[ch].enabled) {
    Serial.print(JOINTS[ch].name);
    Serial.println(" is DISABLED.");
    return false;
  }
  if (!js[ch].attached) {
    Serial.print("ERROR: ");
    Serial.print(JOINTS[ch].name);
    Serial.println(" failed to attach.");
    return false;
  }
  return true;
}

bool startEasedMove(uint8_t ch, float requestedDeg, bool quiet, float scale) {
  if (!jointUsable(ch)) return false;

  JointState& s = js[ch];
  float target = clampToJoint(ch, requestedDeg);
  if (!quiet && fabsf(target - requestedDeg) > 0.01f) {
    Serial.print("Note: ");
    Serial.print(JOINTS[ch].name);
    Serial.print(" limited to ");
    Serial.print(JOINTS[ch].minDeg, 0);
    Serial.print("-");
    Serial.print(JOINTS[ch].maxDeg, 0);
    Serial.print(" deg, using ");
    Serial.print(target, 0);
    Serial.println(" deg.");
  }

  s.startDeg  = s.currentDeg;
  s.targetDeg = target;
  s.easing    = activeEasing;
  s.distance  = fabsf(target - s.startDeg);

  if (s.distance <= 0.05f) {
    s.currentDeg  = target;
    s.durationSec = 0.0f;
    s.moving      = false;
    writeJointUs(ch, degToUs(ch, target));
    return true;
  }

  setParked(false);

  scale = constrain(scale, 0.01f, 1.0f);
  float vMax = s.speed * scale;
  float aMax = s.accel * scale * scale;

  if (s.easing == EASE_SCURVE) {
    planSCurve(s, vMax, aMax);
  } else {
    s.durationSec = moveDurationSec(s.easing, s.distance, vMax, aMax);
  }
  if (s.durationSec < 0.004f) s.durationSec = 0.004f;

  s.moveStartUs = micros();
  s.moving      = true;

  if (!quiet) {
    Serial.print("Moving ");
    Serial.print(JOINTS[ch].name);
    Serial.print(" ");
    Serial.print(s.startDeg, 0);
    Serial.print(" -> ");
    Serial.print(s.targetDeg, 0);
    Serial.print(" deg in ");
    Serial.print(s.durationSec, 2);
    Serial.println(" s");
  }
  return true;
}

// All listed joints start and finish together. targets[i] < 0 (KEEP) = skip.
float startSyncedMove(const float targets[NUM_SERVOS], float rate) {
  float dur[NUM_SERVOS];
  float maxDur = 0.0f;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    dur[i] = 0.0f;
    if (targets[i] < 0.0f || !JOINTS[i].enabled || !js[i].attached) continue;
    float dist = fabsf(clampToJoint(i, targets[i]) - js[i].currentDeg);
    dur[i] = moveDurationSec(activeEasing, dist, js[i].speed * rate, js[i].accel * rate * rate);
    if (dur[i] > maxDur) maxDur = dur[i];
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (targets[i] < 0.0f || !JOINTS[i].enabled || !js[i].attached) continue;
    float stretch = (maxDur > 0.0f && dur[i] > 0.0f) ? (dur[i] / maxDur) : 1.0f;
    startEasedMove(i, targets[i], true, rate * stretch);
  }
  return maxDur;
}

void stopAllJoints() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (js[i].moving) {
      js[i].moving    = false;
      js[i].targetDeg = js[i].currentDeg;
    }
  }
}

void updateServosEased() {
  unsigned long now = micros();
  if (now - lastUpdateUs < UPDATE_INTERVAL_US) return;
  lastUpdateUs = now;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    JointState& s = js[i];
    if (!s.moving || !s.attached) continue;

    float t = (float)(now - s.moveStartUs) / 1000000.0f;

    if (t >= s.durationSec) {
      s.currentDeg = s.targetDeg;
      s.moving     = false;
      writeJointUs(i, degToUs(i, s.currentDeg));
      savePosition(i, false);
      continue;
    }

    float direction = (s.targetDeg >= s.startDeg) ? 1.0f : -1.0f;
    float travelled = (s.easing == EASE_SCURVE)
                        ? scurvePosition(s, t)
                        : s.distance * evaluateEasing(t / s.durationSec, s.easing);

    s.currentDeg = s.startDeg + direction * travelled;
    writeJointUs(i, degToUs(i, s.currentDeg));
  }
}

void checkParkComplete() {
  if (!parkPending || anyMoving()) return;
  parkPending = false;
  saveAllPositions();
  setParked(true);
  Serial.println("PARKED at home. It is now safe to switch off the power.");
}

// ---------------------------------------------------------------------------
// Sequencer
// ---------------------------------------------------------------------------
float stepRate(uint8_t idx) {
  float stepPct = constrain((int)SEQUENCE[idx].speedPct, 10, 100);
  return ((float)playRatePct / 100.0f) * (stepPct / 100.0f);
}

void runStep(uint8_t idx) {
  float t = startSyncedMove(SEQUENCE[idx].deg, stepRate(idx));
  Serial.print("Move -> [");
  Serial.print(SEQUENCE[idx].label);
  Serial.print("] (");
  Serial.print(t, 2);
  Serial.println(" s)");
}

// ---------------------------------------------------------------------------
// Link to base controller
// ---------------------------------------------------------------------------
uint8_t readCode() {
  return (digitalRead(CODE_PIN_HI) << 1) | digitalRead(CODE_PIN_LO);
}

void startCode(uint8_t code) {
  activeCode  = code;
  parkPending = false;
  stopAllJoints();
  digitalWrite(DONE_OUT, LOW);               // busy - must go LOW right away
  segIdx    = SEGMENTS[code].first;
  segActive = true;
  holding   = false;
  seqIndex  = segIdx;
  Serial.print("CODE ");
  Serial.print(code >> 1);
  Serial.print(code & 1);
  Serial.print(" ");
  Serial.println(CODE_NAME[code]);
  runStep(segIdx);
}

void linkUpdate() {
  // A new code must stay steady for CODE_STABLE_MS. Going 01 -> 10 flips both
  // bits, and the wires may not change at exactly the same moment.
  uint8_t c = readCode();
  if (c != candCode) {
    candCode  = c;
    candSince = millis();
  } else if ((int8_t)c != activeCode && millis() - candSince >= CODE_STABLE_MS) {
    startCode(c);
  }

  if (!segActive || anyMoving()) return;

  if (!holding) {                            // step finished -> apply its holdMs
    holding   = true;
    holdUntil = millis() + SEQUENCE[segIdx].holdMs;
    return;
  }
  if ((long)(millis() - holdUntil) < 0) return;
  holding = false;

  if (segIdx < SEGMENTS[activeCode].last) {
    segIdx++;
    seqIndex = segIdx;
    runStep(segIdx);
  } else {
    segActive = false;
    digitalWrite(DONE_OUT, HIGH);            // whole segment finished
    Serial.println("DONE -> HIGH");
  }
}

// ---------------------------------------------------------------------------
// Status / help
// ---------------------------------------------------------------------------
void printSequence() {
  Serial.print("Hardcoded sequence: ");
  Serial.print(SEQ_COUNT);
  Serial.print(" states | rate ");
  Serial.print(playRatePct);
  Serial.println("%");
  for (uint8_t k = 0; k < SEQ_COUNT; k++) {
    Serial.print("  ");
    Serial.print(k + 1);
    Serial.print(". ");
    Serial.print(SEQUENCE[k].label);
    Serial.print(" ->");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.print(" ");
      if (SEQUENCE[k].deg[i] < 0.0f) Serial.print("-");
      else                           Serial.print(SEQUENCE[k].deg[i], 0);
    }
    Serial.println();
  }
}

void printStatus() {
  Serial.print("Profile: ");
  Serial.print(getEasingName(activeEasing));
  Serial.print(" | Parked: ");
  Serial.println(parkedFlag ? "YES" : "NO");

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    const JointState& s = js[i];
    Serial.print("  Ch ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(JOINTS[i].name);
    Serial.print(": ");
    if (!JOINTS[i].enabled) { Serial.println("DISABLED");      continue; }
    if (!s.attached)        { Serial.println("ATTACH FAILED"); continue; }
    Serial.print(s.currentDeg, 1);
    Serial.print(" deg | home ");
    Serial.print(s.homeDeg, 1);
    Serial.println(s.moving ? " [moving]" : "");
  }
}

void printWhere() {
  uint8_t c = readCode();
  Serial.print("Code on pins: ");
  Serial.print(c >> 1);
  Serial.print(c & 1);
  Serial.print(" ");
  Serial.print(CODE_NAME[c]);
  Serial.print(" | running code: ");
  if (activeCode < 0) Serial.print("none");
  else                Serial.print(CODE_NAME[activeCode]);
  Serial.print(" | segment ");
  Serial.print(segActive ? "ACTIVE" : "idle");
  Serial.print(" | last state: ");
  Serial.print(seqIndex + 1);
  Serial.print(" | DONE ");
  Serial.println(digitalRead(DONE_OUT) ? "HIGH" : "LOW");
}

void dumpPose() {
  if (anyMoving()) {
    Serial.println("Wait until the arm stops moving.");
    return;
  }
  Serial.println("Paste this line into SEQUENCE[]:");
  Serial.print("  { { ");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print((int)roundf(js[i].currentDeg));
    if (i < NUM_SERVOS - 1) Serial.print(", ");
  }
  Serial.println(" },   500,   100,   \"State X\" },");
}

void printHelp() {
  Serial.println("------------------------------------------------------------");
  Serial.println("Channels: 0 Base | 1 Shoulder | 2 Elbow | 3 Wrist Pitch | 4 Wrist Roll | 5 Gripper");
  Serial.println("Manual Stepping (cancels the segment started by the base):");
  Serial.println("  next | prev              Step forward or backward through SEQUENCE[] states");
  Serial.println("  test [1-8]               Jump to a specific state in the sequence");
  Serial.println("  stop                     Halt all motors instantly");
  Serial.println("  rate [10-100]            Global playback speed %");
  Serial.println("Teaching & Tuning:");
  Serial.println("  1+ / 1-  |  j [ch] [deg] |  step [deg]  |  [ch] [angle]");
  Serial.println("  dump                     Print current pose as a SEQUENCE[] line");
  Serial.println("Home & park:");
  Serial.println("  home | park | sethome | resethome");
  Serial.println("Other:");
  Serial.println("  status | where | help | list | ease [scurve|quintic|cubic|sine|quad]");
  Serial.println("------------------------------------------------------------");
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
void jogJoint(int ch, float delta) {
  if (ch < 0 || ch >= NUM_SERVOS) {
    Serial.println("Invalid channel. Use 0 through 5.");
    return;
  }
  startEasedMove((uint8_t)ch, js[ch].targetDeg + delta, false, 1.0f);
}

void handleSerialCommands() {
  if (Serial.available() == 0) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  line.toLowerCase();

  // Basic info commands
  if (line == "help")   { printHelp();     return; }
  if (line == "status") { printStatus();   return; }
  if (line == "where")  { printWhere();    return; }
  if (line == "list")   { printSequence(); return; }
  if (line == "dump")   { dumpPose();      return; }

  // Manual stepping
  if (line == "next" || line == "previous" || line == "prev") {
    segActive   = false;
    parkPending = false;
    stopAllJoints();

    if (line == "next") {
      seqIndex = (seqIndex + 1) % SEQ_COUNT;
    } else {
      if (seqIndex == 0) seqIndex = SEQ_COUNT - 1;
      else seqIndex--;
    }

    runStep(seqIndex);
    return;
  }

  if (line.startsWith("test")) {
    int n = 0;
    if (sscanf(line.c_str(), "test %d", &n) != 1 || n < 1 || n > SEQ_COUNT) {
      Serial.print("Usage: test [1-");
      Serial.print(SEQ_COUNT);
      Serial.println("]");
      return;
    }
    segActive   = false;
    parkPending = false;
    stopAllJoints();
    seqIndex = n - 1;
    runStep(seqIndex);
    return;
  }

  if (line == "stop") {
    segActive   = false;
    parkPending = false;
    stopAllJoints();
    saveAllPositions();
    Serial.println("All joints stopped.");
    return;
  }

  if (line.startsWith("rate")) {
    int v = 0;
    if (sscanf(line.c_str(), "rate %d", &v) != 1 || v < 10 || v > 100) {
      Serial.println("Usage: rate [10-100]");
    } else {
      playRatePct = v;
      Serial.print("Playback rate = ");
      Serial.print(playRatePct);
      Serial.println("% (applies to next move)");
    }
    return;
  }

  if (line.length() == 2 && isDigit(line[0]) && (line[1] == '+' || line[1] == '-')) {
    int ch = line[0] - '0';
    jogJoint(ch, (line[1] == '+') ? (float)jogStepDeg : -(float)jogStepDeg);
    return;
  }

  if (line.startsWith("j ")) {
    int ch = -1, delta = 0;
    if (sscanf(line.c_str(), "j %d %d", &ch, &delta) != 2) {
      Serial.println("Usage: j [ch] [deg], e.g. 'j 1 -3'");
      return;
    }
    jogJoint(ch, (float)delta);
    return;
  }

  if (line.startsWith("step")) {
    int v = 0;
    if (sscanf(line.c_str(), "step %d", &v) != 1 || v < 1 || v > 45) {
      Serial.println("Usage: step [1-45]");
      return;
    }
    jogStepDeg = v;
    Serial.print("Jog step = ");
    Serial.print(jogStepDeg);
    Serial.println(" deg");
    return;
  }

  if (line == "resethome") {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      js[i].homeDeg = clampToJoint(i, JOINTS[i].homeDeg);
      saveHome(i);
    }
    Serial.println("Home angles restored to defaults.");
    return;
  }

  if (line == "sethome") {
    if (anyMoving()) {
      Serial.println("Wait until the arm stops moving.");
      return;
    }
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      if (!JOINTS[i].enabled || !js[i].attached) continue;
      js[i].homeDeg = clampToJoint(i, js[i].currentDeg);
      saveHome(i);
    }
    Serial.println("Current pose saved as home.");
    return;
  }

  if (line == "home" || line == "park") {
    segActive = false;
    bool isPark = (line == "park");
    Serial.println(isPark ? "Parking: moving to home..." : "Moving to home...");
    float targets[NUM_SERVOS];
    for (uint8_t i = 0; i < NUM_SERVOS; i++) targets[i] = js[i].homeDeg;
    startSyncedMove(targets, 1.0f);
    if (isPark) parkPending = true;
    return;
  }

  if (line.startsWith("ease")) {
    String mode = line.substring(4);
    mode.trim();
    if      (mode == "scurve")  activeEasing = EASE_SCURVE;
    else if (mode == "quintic") activeEasing = EASE_QUINTIC_IN_OUT;
    else if (mode == "cubic")   activeEasing = EASE_CUBIC_IN_OUT;
    else if (mode == "sine")    activeEasing = EASE_SINE_IN_OUT;
    else if (mode == "quad")    activeEasing = EASE_QUAD_IN_OUT;
    else {
      Serial.println("Unknown profile. Choose: scurve, quintic, cubic, sine, or quad.");
      return;
    }
    Serial.print("Profile set to: ");
    Serial.println(getEasingName(activeEasing));
    return;
  }

  int ch = -1, value = 0;
  if (sscanf(line.c_str(), "%d %d", &ch, &value) == 2) {
    if (ch < 0 || ch >= NUM_SERVOS) {
      Serial.println("Invalid channel. Use 0 through 5.");
      return;
    }
    startEasedMove((uint8_t)ch, (float)value, false, 1.0f);
    return;
  }

  Serial.println("Invalid command. Type 'help' for the list.");
}

// ---------------------------------------------------------------------------
void setup() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    pinMode(JOINTS[i].pin, OUTPUT);
    digitalWrite(JOINTS[i].pin, LOW);
  }

  // Link pins: DONE starts LOW (busy) so the base never sees a false DONE
  pinMode(CODE_PIN_HI, INPUT_PULLDOWN);
  pinMode(CODE_PIN_LO, INPUT_PULLDOWN);
  pinMode(DONE_OUT, OUTPUT);
  digitalWrite(DONE_OUT, LOW);

  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  prefs.begin("arm", false);
  parkedFlag  = prefs.getBool("parked", false);
  playRatePct = constrain(DEFAULT_RATE_PCT, 10, 100);

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    const JointConfig& j = JOINTS[i];
    JointState& s = js[i];
    char key[6];

    keyFor(key, sizeof(key), 'h', i);
    s.homeDeg = clampToJoint(i, prefs.getFloat(key, j.homeDeg));

    keyFor(key, sizeof(key), 'p', i);
    s.savedPosDeg = clampToJoint(i, prefs.getFloat(key, s.homeDeg));

    float startDeg = parkedFlag ? s.homeDeg : s.savedPosDeg;

    s.startDeg    = startDeg;
    s.currentDeg  = startDeg;
    s.targetDeg   = startDeg;
    s.speed       = j.maxSpeed;
    s.accel       = j.maxAccel;
    s.easing      = activeEasing;
    s.moveStartUs = 0;
    s.durationSec = 0.0f;
    s.distance    = 0.0f;
    s.peakVel     = 0.0f;
    s.rampSec     = 0.0f;
    s.cruiseSec   = 0.0f;
    s.lastUs      = -1;
    s.moving      = false;
    s.attached    = false;
  }

  Serial.println();
  Serial.println(parkedFlag ? "=== Arm was PARKED - holding home pose ==="
                            : "=== Arm was NOT parked - holding last saved positions ===");

  for (uint8_t k = 0; k < NUM_SERVOS; k++) {
    uint8_t i = STARTUP_ORDER[k];
    const JointConfig& j = JOINTS[i];
    JointState& s = js[i];

    Serial.print("  Ch ");
    Serial.print(i);
    Serial.print(" [");
    Serial.print(j.name);
    Serial.print("] -> ");

    if (!j.enabled) {
      Serial.println("DISABLED");
      continue;
    }

    armServo[i].setPeriodHertz(j.freqHz);
    armServo[i].attach(j.pin, j.minPulseUs, j.maxPulseUs);
    if (!armServo[i].attached()) {
      Serial.println("ATTACH FAILED");
      continue;
    }

    s.attached = true;
    writeJointUs(i, degToUs(i, s.currentDeg));
    Serial.print("holding ");
    Serial.print(s.currentDeg, 1);
    Serial.println(" deg");

    delay(ATTACH_STAGGER_MS);
  }

  lastUpdateUs = micros();
  Serial.println("Ready. Waiting for the base code (or use 'next' / 'test <n>' manually).");
  printHelp();
}

// ---------------------------------------------------------------------------
void loop() {
  handleSerialCommands();
  linkUpdate();
  updateServosEased();
  checkParkComplete();
  delay(1);
}

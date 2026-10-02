/*
  ============================================================================
  6-Servo Arm  -  ARM 1  -  controlled by the BASE (main) ESP32
  Board: ESP32
  Library: "ESP32Servo"  (Preferences is built into the ESP32 core)

  The base ESP32 sends a 2-bit code on two wires. Each code plays one part of
  the SEQUENCE[] table below, once, then the arm waits for the next code:

    00 HOME      -> State 1       original pose
    01 PICK UP   -> States 2-5    above object, lower, close gripper, lift
    10 PUT DOWN  -> States 6-7    move to drop point, lower / release
    11 BACK HOME -> State 8       back to the original pose

  Change the ranges in PHASE[] if your states belong to other phases.

  WIRING to the base (GND of both ESP32 must be connected together):
    base COM_PIN_R1_1 (GPIO 12)  ->  CODE_PIN_1  (GPIO 32)   bit 1, left digit
    base COM_PIN_R1_2 (GPIO 13)  ->  CODE_PIN_2  (GPIO 22)   bit 0, right digit
    DONE_OUT_PIN (GPIO 23)       ->  base DONE_PIN_1 (GPIO 18)
  (Arm 2 uses base COM_PIN_R2_x + DONE_PIN_2, Arm 3 uses COM_PIN_R3_x + DONE_PIN_3.
   The same file works on all three arms - only the SEQUENCE angles differ.)

  DONE goes HIGH when the arm has finished the moves for the current code, and
  drops LOW as soon as a new code arrives.
  Safety: if code 11 (pogo pin connected) arrives while the arm is still
  putting the object down (code 10), the arm finishes the put-down first and
  then goes home.

  Serial Monitor still works for testing: type 'help'.
  Tip: jog the arm to a pose and type 'dump' - it prints that pose as a
  ready-to-paste line for the table.

  Servos:
    Ch 0  Base         250Hz   GPIO 13
    Ch 1  Shoulder     250Hz   GPIO 14   (35kg TD-8135MG)
    Ch 2  Elbow        250Hz   GPIO 27   (25kg TD-8125MG)
    Ch 3  Wrist Pitch   50Hz   GPIO 26
    Ch 4  Wrist Roll    50Hz   GPIO 25
    Ch 5  Gripper       50Hz   GPIO 33

  WIRING: Servo PSU (-) MUST be connected to ESP32 GND (common ground).
  ============================================================================
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <math.h>

const uint8_t NUM_SERVOS = 6;

// ===========================================================================
//                        >>> SEQUENCE SETTINGS <<<
// ===========================================================================
const float KEEP = -1.0f;   // Use KEEP to leave a joint where it is

struct SeqStep {
  float       deg[NUM_SERVOS];  // Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper
  uint32_t    holdMs;           // Pause after arriving at this state
  uint8_t     speedPct;         // Speed of the move INTO this state (10-100 %)
  const char* label;
};

// Your pick-and-place states for ARM 1.
const SeqStep SEQUENCE[] = {
  //   Base  Shldr  Elbow  WPitch WRoll  Grip     hold ms  speed%  label
  { {114, 0, 0, 105, 12, 20},   1000,   100,   "State 1: Home (gripper open)" },
  { {114, 0, 0, 105, 12, 150},    300,   100,   "State 2: Above object"        },
  { {114, 80, 62, 105, 12, 150},    300,    50,   "State 3: Lower to object"     },
  { {164, 80, 62, 105, 12, 150},    500,   100,   "State 4: Close gripper"       },
  { {164, 25, 24, 105, 12, 150},    300,    60,   "State 5: Lift"                },
  { {164, 25, 24, 105, 12, 20},    300,   100,   "State 6: Move to drop point"  },
  { {164, 80, 62, 105, 12, 20},    500,   100,   "State 7: Release"             },
  { {114, 0, 0, 101, 12, 20},    1000,   100,   "State 8: Back to Home"             }
};
const uint8_t SEQ_COUNT = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// ---------------------------------------------------------------------------
// Link to the BASE ESP32: which states each code plays (1-based, inclusive)
// ---------------------------------------------------------------------------
struct CodePhase {
  uint8_t     firstStep;
  uint8_t     lastStep;
  const char* name;
};

const CodePhase PHASE[4] = {
  { 1, 1, "00 HOME"      },   // State 1
  { 2, 5, "01 PICK UP"   },   // States 2-5
  { 6, 7, "10 PUT DOWN"  },   // States 6-7
  { 8, 8, "11 BACK HOME" }    // State 8
};

const bool CODE_CONTROL = true;     // false = ignore the base, Serial control only

#define CODE_PIN_1   32             // <- base COM_PIN_R1_1 (bit 1)
#define CODE_PIN_2   22             // <- base COM_PIN_R1_2 (bit 0)
#define DONE_OUT_PIN 23             // -> base DONE_PIN_1

const unsigned long CODE_STABLE_MS = 30;   // code must be steady this long (noise filter)

const bool          AUTO_START          = false;  // The base starts the arm, not power-up
const unsigned long AUTO_START_DELAY_MS = 3000;   // Used only if AUTO_START = true
const bool          LOOP_FOREVER        = true;   // Used only by 'run'/'loop'/AUTO_START
const long          LOOP_COUNT          = 1;      // Used only if LOOP_FOREVER = false
const int           DEFAULT_RATE_PCT    = 100;    // Global playback speed (10-100 %)
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
  // name                         en    pin freq minUs maxUs  min    max    home   speed   accel
  { "Base",                       true, 13, 250, 500, 2500,  0.0f, 180.0f, 90.0f,  60.0f, 80.0f },
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

enum SeqState : uint8_t { SEQ_IDLE, SEQ_MOVING, SEQ_HOLD, SEQ_PAUSED };

Servo       armServo[NUM_SERVOS];
JointState  js[NUM_SERVOS];
Preferences prefs;

SeqState      seqState       = SEQ_IDLE;
uint8_t       seqIndex       = 0;
long          loopsRemaining = 0;      // -1 = forever
long          loopNumber     = 0;
unsigned long stateStartMs   = 0;
int           playRatePct    = DEFAULT_RATE_PCT;
bool          autoPending    = false;
unsigned long autoAtMs       = 0;

// Code-phase state (link to the base ESP32)
bool          phaseMode      = false;  // true = playing one code phase only
uint8_t       phaseLast      = 0;      // last SEQUENCE index of the running phase
int           phaseCode      = -1;     // code of the running / last finished phase
bool          phaseDone      = false;  // phase finished -> DONE wire HIGH
int           deferredCode   = -1;     // code 11 waiting for the put-down to finish
int           lastCode       = -1;     // last accepted code
int           pendingCode    = -1;     // code being filtered
unsigned long pendingSinceMs = 0;

bool          parkedFlag   = false;
bool          parkPending  = false;
int           jogStepDeg   = 5;
unsigned long lastUpdateUs = 0;

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

bool sequenceActive() {
  return seqState != SEQ_IDLE;
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
  if (!forceWrite && sequenceActive()) return;   // Protect flash during playback
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

      if (!sequenceActive()) {
        Serial.print("Reached -> ");
        Serial.print(JOINTS[i].name);
        Serial.print(": ");
        Serial.print(s.currentDeg, 1);
        Serial.println(" deg");
      }
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
// Sequence player
// ---------------------------------------------------------------------------
float stepRate(uint8_t idx) {
  float stepPct = constrain((int)SEQUENCE[idx].speedPct, 10, 100);
  return ((float)playRatePct / 100.0f) * (stepPct / 100.0f);
}

void runStep(uint8_t idx) {
  float t = startSyncedMove(SEQUENCE[idx].deg, stepRate(idx));
  stateStartMs = millis();
  seqState     = SEQ_MOVING;

  Serial.print("[");
  if (loopsRemaining != 0) {
    Serial.print("Loop ");
    Serial.print(loopNumber);
    Serial.print(" | ");
  }
  Serial.print(SEQUENCE[idx].label);
  Serial.print("] move ");
  Serial.print(t, 2);
  Serial.print(" s, hold ");
  Serial.print((unsigned long)SEQUENCE[idx].holdMs);
  Serial.println(" ms");
}

void finishSequence(const char* reason) {
  seqState       = SEQ_IDLE;
  loopsRemaining = 0;
  phaseMode      = false;
  phaseDone      = false;
  deferredCode   = -1;
  saveAllPositions();
  Serial.println(reason);
}

// Play only the states of one code (00/01/10/11), once
void startPhase(int code) {
  if (SEQ_COUNT == 0) {
    Serial.println("SEQUENCE[] is empty.");
    return;
  }
  int first = (int)PHASE[code].firstStep - 1;
  int last  = (int)PHASE[code].lastStep  - 1;
  if (last >= (int)SEQ_COUNT) last = SEQ_COUNT - 1;
  if (first > last)           first = last;
  if (first < 0)              first = 0;

  stopAllJoints();
  parkPending    = false;
  autoPending    = false;
  phaseMode      = true;
  phaseCode      = code;
  phaseLast      = (uint8_t)last;
  phaseDone      = false;
  seqIndex       = (uint8_t)first;
  loopsRemaining = 0;
  loopNumber     = 1;

  Serial.printf(">>> PHASE %s: states %d-%d\n", PHASE[code].name, first + 1, last + 1);
  runStep(seqIndex);
}

void finishPhase() {
  seqState       = SEQ_IDLE;
  phaseMode      = false;
  loopsRemaining = 0;
  saveAllPositions();
  Serial.printf(">>> PHASE %s finished\n", PHASE[phaseCode].name);

  if (deferredCode >= 0) {            // code 11 arrived during the put-down
    int c = deferredCode;
    deferredCode = -1;
    startPhase(c);
  } else {
    phaseDone = true;                 // DONE wire goes HIGH
  }
}

void advanceSequence() {
  seqIndex++;

  if (phaseMode && seqIndex > phaseLast) {
    finishPhase();
    return;
  }

  if (seqIndex >= SEQ_COUNT) {
    if (loopsRemaining < 0) {
      loopNumber++;
      seqIndex = 0;
    } else if (loopsRemaining > 1) {
      loopsRemaining--;
      loopNumber++;
      seqIndex = 0;
    } else {
      finishSequence("Sequence finished.");
      return;
    }
  }
  runStep(seqIndex);
}

void startSequence(long loops) {
  if (SEQ_COUNT == 0) {
    Serial.println("SEQUENCE[] is empty.");
    return;
  }
  stopAllJoints();
  parkPending    = false;
  autoPending    = false;
  phaseMode      = false;
  phaseDone      = false;
  deferredCode   = -1;
  seqIndex       = 0;
  loopsRemaining = loops;
  loopNumber     = 1;

  Serial.print("Running ");
  Serial.print(SEQ_COUNT);
  Serial.print(" states ");
  if (loops < 0) {
    Serial.println("FOREVER (type 'stop' to end)");
  } else {
    Serial.print(loops);
    Serial.println(loops == 1 ? " time" : " times");
  }
  runStep(0);
}

void updateSequence() {
  unsigned long now = millis();

  if (autoPending && (long)(now - autoAtMs) >= 0) {
    autoPending = false;
    Serial.println("AUTO-START");
    startSequence(LOOP_FOREVER ? -1 : LOOP_COUNT);
    return;
  }

  switch (seqState) {
    case SEQ_MOVING:
      if (!anyMoving()) {
        stateStartMs = now;
        seqState     = SEQ_HOLD;
      }
      break;
    case SEQ_HOLD:
      if (now - stateStartMs >= SEQUENCE[seqIndex].holdMs) advanceSequence();
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Link to the base ESP32: read the 2-bit code, drive the DONE wire
// ---------------------------------------------------------------------------
int readCode() {
  return (digitalRead(CODE_PIN_1) << 1) | digitalRead(CODE_PIN_2);   // bit1 = COM1, bit0 = COM2
}

void onNewCode(int code) {
  Serial.printf(">>> CODE %d%d received\n", code >> 1, code & 1);

  // Safety: the pogo pin (code 11) must not cut the put-down (10) in half.
  if (code == 3 && phaseMode && phaseCode == 2) {
    deferredCode = 3;
    Serial.println("    11 waits until the put-down has finished");
    return;
  }

  deferredCode = -1;
  startPhase(code);
}

void updateCodeInput() {
  if (!CODE_CONTROL) return;

  int code = readCode();
  if (code != pendingCode) {                 // changed: restart the noise filter
    pendingCode    = code;
    pendingSinceMs = millis();
    return;
  }
  if (millis() - pendingSinceMs < CODE_STABLE_MS) return;
  if (code == lastCode) return;

  lastCode = code;
  onNewCode(code);
}

void updateDoneOutput() {
  static int lastOut = -1;
  bool done = phaseDone && !sequenceActive() && deferredCode < 0;
  if ((int)done != lastOut) {
    lastOut = (int)done;
    digitalWrite(DONE_OUT_PIN, done ? HIGH : LOW);
    Serial.println(done ? "DONE -> HIGH" : "DONE -> LOW");
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
    Serial.print(" | hold ");
    Serial.print((unsigned long)SEQUENCE[k].holdMs);
    Serial.print(" ms | speed ");
    Serial.print(SEQUENCE[k].speedPct);
    Serial.println("%");
  }
  Serial.println("Code phases (from the base ESP32):");
  for (uint8_t c = 0; c < 4; c++) {
    Serial.print("  ");
    Serial.print(PHASE[c].name);
    Serial.print(" -> states ");
    Serial.print(PHASE[c].firstStep);
    Serial.print("-");
    Serial.println(PHASE[c].lastStep);
  }
}

void printStatus() {
  int code = readCode();
  Serial.print("Code in: ");
  Serial.print(code >> 1);
  Serial.print(code & 1);
  Serial.print(" | DONE out: ");
  Serial.println(digitalRead(DONE_OUT_PIN) ? "HIGH" : "LOW");

  Serial.print("Profile: ");
  Serial.print(getEasingName(activeEasing));
  Serial.print(" | Parked: ");
  Serial.print(parkedFlag ? "YES" : "NO");
  Serial.print(" | Sequence: ");
  switch (seqState) {
    case SEQ_IDLE:   Serial.println("idle");   break;
    case SEQ_PAUSED: Serial.println("PAUSED"); break;
    default:
      Serial.print("running state ");
      Serial.println(seqIndex + 1);
      break;
  }
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

// Prints the current pose as a line ready to paste into SEQUENCE[]
void dumpPose() {
  if (anyMoving()) {
    Serial.println("Wait until the arm stops moving.");
    return;
  }
  Serial.println("Paste this line into SEQUENCE[] (edit the hold, speed and label):");
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
  Serial.println("The base ESP32 drives this arm with codes 00 / 01 / 10 / 11.");
  Serial.println("Sequence:");
  Serial.println("  run | run [N] | loop     Play once / N times / forever");
  Serial.println("  pause | resume | stop");
  Serial.println("  rate [10-100]            Global playback speed %");
  Serial.println("  test [1-8]               Move to one state only");
  Serial.println("  list                     Show the states and the code phases");
  Serial.println("Teaching:");
  Serial.println("  1+ / 1-  |  j [ch] [deg]  |  step [deg]  |  [ch] [angle]");
  Serial.println("  dump                     Print current pose as a SEQUENCE[] line");
  Serial.println("Home & park:");
  Serial.println("  home | park | sethome | resethome");
  Serial.println("Other:");
  Serial.println("  status | help | ease [scurve|quintic|cubic|sine|quad]");
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

  // ---- Allowed at any time ----
  if (line == "help")   { printHelp();     return; }
  if (line == "status") { printStatus();   return; }
  if (line == "list")   { printSequence(); return; }

  if (line == "stop") {
    bool wasRunning = sequenceActive() || autoPending;
    autoPending = false;
    parkPending = false;
    stopAllJoints();
    if (wasRunning) {
      finishSequence("Sequence STOPPED. Type 'run' or 'loop' to start again.");
    } else {
      saveAllPositions();
      Serial.println("All joints stopped.");
    }
    return;
  }

  if (line == "pause") {
    if (seqState == SEQ_MOVING || seqState == SEQ_HOLD) {
      stopAllJoints();
      seqState = SEQ_PAUSED;
      Serial.print("PAUSED at state ");
      Serial.print(seqIndex + 1);
      Serial.println(". Type 'resume' or 'stop'.");
    } else {
      Serial.println("Nothing is running.");
    }
    return;
  }

  if (line == "resume") {
    if (seqState == SEQ_PAUSED) {
      Serial.println("Resuming...");
      runStep(seqIndex);
    } else {
      Serial.println("Sequence is not paused.");
    }
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
      Serial.println("% (applies from the next state)");
    }
    return;
  }

  // ---- Everything below is blocked while the sequence runs ----
  if (sequenceActive() || autoPending) {
    Serial.println("Sequence is running. Type 'stop' (or 'pause') first.");
    return;
  }

  if (line == "run")  { startSequence(1);  return; }
  if (line == "loop") { startSequence(-1); return; }
  if (line.startsWith("run")) {
    int n = 0;
    if (sscanf(line.c_str(), "run %d", &n) != 1 || n < 1) {
      Serial.println("Usage: run | run [N] | loop");
    } else {
      startSequence(n);
    }
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
    float t = startSyncedMove(SEQUENCE[n - 1].deg, stepRate(n - 1));
    Serial.print("Testing ");
    Serial.print(SEQUENCE[n - 1].label);
    Serial.print(" (");
    Serial.print(t, 2);
    Serial.println(" s)");
    return;
  }

  if (line == "dump") { dumpPose(); return; }

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

  // Link to the base ESP32
  pinMode(CODE_PIN_1, INPUT_PULLDOWN);
  pinMode(CODE_PIN_2, INPUT_PULLDOWN);
  pinMode(DONE_OUT_PIN, OUTPUT);
  digitalWrite(DONE_OUT_PIN, LOW);

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
  Serial.println("Ready.");
  printHelp();
  printSequence();

  if (CODE_CONTROL) {
    Serial.println("Code control ON: waiting for codes from the base ESP32.");
  }

  if (AUTO_START && SEQ_COUNT > 0) {
    autoPending = true;
    autoAtMs    = millis() + AUTO_START_DELAY_MS;
    Serial.print("AUTO-START in ");
    Serial.print(AUTO_START_DELAY_MS / 1000);
    Serial.println(" s... type 'stop' to cancel.");
  }
}

// ---------------------------------------------------------------------------
void loop() {
  handleSerialCommands();
  updateCodeInput();       // 2-bit code from the base ESP32
  updateServosEased();
  updateSequence();
  updateDoneOutput();      // tell the base when the phase is finished
  checkParkComplete();
  delay(1);
}

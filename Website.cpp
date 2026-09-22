/*
  ============================================================================
  PCA9685 Channels 10-14 Test — 5-Servo Web Jog Interface
  Board: ESP32 (hardware I2C to PCA9685, WiFi access point for control)
  ============================================================================
  Hosts its own WiFi network and a web page with 5 sliders (each paired
  with a typeable number box) for testing the base, shoulder, elbow, wrist
  pitch, and wrist roll servos -- no USB/serial connection needed once
  running.

  SERVO -> PCA9685 CHANNEL MAPPING (channels 10-14, left to right)
  ------------------------------------------------------------------------
  Base         -> channel 10
  Shoulder     -> channel 11
  Elbow        -> channel 12
  Wrist Pitch  -> channel 13
  Wrist Roll   -> channel 14

  Double-check against your board's silkscreen that these printed numbers
  are the actual PCA9685 channel numbers (0-15).

  SERVO CLASSIFICATION
  ------------------------------------------------------------------------
  3 LARGE servos: base, shoulder, elbow       -> LARGE_SERVO_SPEED_DEG_PER_SEC
  2 SMALL servos: wrist pitch, wrist roll     -> SMALL_SERVO_SPEED_DEG_PER_SEC

  HOME POSITION: all 5 servos at 90 degrees. The "Reset to Home" button
  moves all 5 there together (synchronized arrival), using each servo's own
  class speed limit -- the same math used for coordinated moves in the
  autonomous sequence sketch: move duration is set by whichever servo needs
  the most time at ITS OWN speed limit, so no servo is ever rushed past its
  own class cap.

  TWO WAYS TO SET AN ANGLE
  ------------------------------------------------------------------------
  - Drag the slider for live jogging.
  - Type an exact value into the number box next to it and press Enter/tab
    away -- the slider updates to match, and the servo moves there directly
    (still at that servo's own speed limit, not instantly).
  Both controls always stay in sync with each other and with the arm.

  HOW TO CONNECT
  ------------------------------------------------------------------------
  1. Upload this sketch.
  2. Connect your phone to WiFi network "RoboticArm" (password "arm12345").
  3. Open http://192.168.4.1

  WIRING: ESP32 3V3->PCA9685 VCC, GND->GND, GPIO21->SDA, GPIO22->SCL,
  PCA9685 V+ left disconnected, servo power from the separate power bus.
  ============================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
// WiFi Access Point configuration
// ---------------------------------------------------------------------------
const char* AP_SSID     = "RoboticArm";
const char* AP_PASSWORD = "arm12345";   // WPA2 requires 8+ characters

WebServer server(80);

// ---------------------------------------------------------------------------
// I2C configuration (ESP32 hardware I2C peripheral, for the PCA9685)
// ---------------------------------------------------------------------------
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_CLOCK_HZ 400000UL

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---------------------------------------------------------------------------
// Servo configuration -- 5 servos under test, on PCA9685 channels 10-14
// ---------------------------------------------------------------------------
const uint8_t NUM_SERVOS = 5;

enum ServoRole : uint8_t {
  BASE        = 0,   // local array index, NOT the PCA9685 channel number
  SHOULDER    = 1,
  ELBOW       = 2,
  WRIST_PITCH = 3,
  WRIST_ROLL  = 4
};

// Actual PCA9685 channel number for each servo, indexed by local role.
const uint8_t pcaChannel[NUM_SERVOS] = {10, 11, 12, 13, 14};

const char* SERVO_NAMES[NUM_SERVOS] = {
  "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll"
};

// PCA9685 pulse length range per servo, in ticks. CALIBRATE per servo.
const uint16_t servoMinPulse[NUM_SERVOS] = {102, 102, 102, 102, 102};
const uint16_t servoMaxPulse[NUM_SERVOS] = {512, 512, 512, 512, 512};

// Mechanical angle range per servo, in degrees. CALIBRATE to your arm.
const uint16_t angleMin[NUM_SERVOS] = {0, 0, 0, 0, 0};
const uint16_t angleMax[NUM_SERVOS] = {180, 180, 180, 180, 180};

// Home position: all 5 servos at 90 degrees.
const float HOME_POSE[NUM_SERVOS] = {90, 90, 90, 90, 90};

// ---------------------------------------------------------------------------
// Per-servo speed limits, by class (large vs small)
// ---------------------------------------------------------------------------
const float LARGE_SERVO_SPEED_DEG_PER_SEC = 60.0f;  // base, shoulder, elbow
const float SMALL_SERVO_SPEED_DEG_PER_SEC = 90.0f;  // wrist pitch, wrist roll

const float maxSpeed[NUM_SERVOS] = {
  LARGE_SERVO_SPEED_DEG_PER_SEC,   // BASE
  LARGE_SERVO_SPEED_DEG_PER_SEC,   // SHOULDER
  LARGE_SERVO_SPEED_DEG_PER_SEC,   // ELBOW
  SMALL_SERVO_SPEED_DEG_PER_SEC,   // WRIST_PITCH
  SMALL_SERVO_SPEED_DEG_PER_SEC    // WRIST_ROLL
};

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
void     updateMotion();
void     applySynchronizedTarget(const float pose[NUM_SERVOS]);
void     handleRoot();
void     handleSet();
void     handleGoHome();
void     handleNotFound();
String   buildHtmlPage();

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // All 5 servos start at home (90 degrees).
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = HOME_POSE[i];
    targetAngle[i] = HOME_POSE[i];
    moveSpeed[i] = maxSpeed[i];
    lastWrittenAngle[i] = (uint8_t)round(HOME_POSE[i]);
    writeServoAngle(i, HOME_POSE[i]);
    Serial.print(SERVO_NAMES[i]);
    Serial.print(" (channel ");
    Serial.print(pcaChannel[i]);
    Serial.println(") homed to 90 degrees.");
  }
  lastMotionUpdate = millis();

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (!apStarted) {
    Serial.println("ERROR: Failed to start WiFi access point.");
  } else {
    Serial.print("Connect to WiFi \"");
    Serial.print(AP_SSID);
    Serial.println("\" then open http://192.168.4.1");
  }

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/goHome", handleGoHome);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("Web server started.");
}

// ---------------------------------------------------------------------------
void loop() {
  server.handleClient();
  updateMotion();
  delay(5);
}

// ---------------------------------------------------------------------------
void handleRoot() {
  server.send(200, "text/html", buildHtmlPage());
}

// Handles GET /set?ch=<0-4>&angle=<0-180>  (ch is the LOCAL index, 0-4)
void handleSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Missing ch or angle parameter");
    return;
  }

  int i = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();

  if (i < 0 || i >= NUM_SERVOS) {
    server.send(400, "text/plain", "Invalid channel");
    return;
  }

  angle = constrain(angle, (int)angleMin[i], (int)angleMax[i]);
  moveSpeed[i] = maxSpeed[i];   // single-servo jog: this servo's own limit
  targetAngle[i] = (float)angle;

  server.send(200, "text/plain", "OK");
}

// Handles GET /goHome -- synchronized move of all 5 servos to HOME_POSE
void handleGoHome() {
  applySynchronizedTarget(HOME_POSE);
  server.send(200, "text/plain", "Moving home");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------------------------------------------------------------------
// Sets targets for all 5 servos so they arrive together, without any
// servo ever exceeding its own class speed limit. The move duration is
// set by whichever servo needs the most time at ITS OWN maxSpeed; every
// other servo's speed is then distance/duration, which is guaranteed to
// stay at or below its own limit.
void applySynchronizedTarget(const float pose[NUM_SERVOS]) {
  float moveDuration = 0.0f;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float dist = fabs(pose[i] - currentAngle[i]);
    float neededDuration = dist / maxSpeed[i];
    if (neededDuration > moveDuration) moveDuration = neededDuration;
  }
  if (moveDuration < 0.02f) moveDuration = 0.02f;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float dist = fabs(pose[i] - currentAngle[i]);
    moveSpeed[i] = dist / moveDuration;
    targetAngle[i] = pose[i];
  }
}

// ---------------------------------------------------------------------------
void updateMotion() {
  unsigned long now = millis();
  float deltaTime = (now - lastMotionUpdate) / 1000.0f;
  lastMotionUpdate = now;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float maxStep = moveSpeed[i] * deltaTime;

    if (currentAngle[i] < targetAngle[i]) {
      currentAngle[i] = min(currentAngle[i] + maxStep, targetAngle[i]);
    } else if (currentAngle[i] > targetAngle[i]) {
      currentAngle[i] = max(currentAngle[i] - maxStep, targetAngle[i]);
    }

    uint8_t angleToWrite = (uint8_t)round(currentAngle[i]);
    if (angleToWrite != lastWrittenAngle[i]) {
      writeServoAngle(i, currentAngle[i]);
      lastWrittenAngle[i] = angleToWrite;
    }
  }
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

// Writes an angle to a servo, translating the local index (0-4) into the
// real PCA9685 channel number (10-14) at the point of talking to the chip.
void writeServoAngle(uint8_t servoIndex, float angleDeg) {
  uint16_t pulse = angleToPulse(servoIndex, angleDeg);
  pwm.setPWM(pcaChannel[servoIndex], 0, pulse);
}

// ---------------------------------------------------------------------------
// Builds the HTML page: a Reset to Home button, and 5 servo rows each with
// a slider and a paired number box, kept in sync with each other.
String buildHtmlPage() {
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>5-Servo Jog Test</title>
  <style>
    body { font-family: sans-serif; background:#f5f5f5; margin:0; padding:20px; }
    h1 { font-size: 20px; text-align:center; }
    .joint { background:white; border-radius:10px; padding:15px; margin-bottom:15px;
             box-shadow:0 1px 3px rgba(0,0,0,0.15); }
    .joint label { font-weight:bold; display:flex; justify-content:space-between; }
    .joint .chan { font-weight:normal; color:#888; font-size:12px; }
    .controls { display:flex; align-items:center; gap:10px; margin-top:10px; }
    input[type=range] { flex:1; }
    input[type=number] { width:60px; text-align:center; padding:6px; font-size:14px; }
    #homeBtn { display:block; width:100%; padding:14px; font-size:16px; border:none;
               border-radius:8px; color:white; background:#1565c0; margin-bottom:15px; }
  </style>
</head>
<body>
  <h1>5-Servo Jog Test (channels 10-14)</h1>
  <button id="homeBtn" onclick="goHome()">Reset to Home (90&deg;)</button>
)HTML";

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    int currentInt = (int)round(currentAngle[i]);
    html += "<div class=\"joint\">";
    html += "<label>" + String(SERVO_NAMES[i]) + " <span class=\"chan\">ch " + String(pcaChannel[i]) + "</span></label>";
    html += "<div class=\"controls\">";
    html += "<input type=\"range\" id=\"slider" + String(i) + "\" min=\"" + String(angleMin[i]) + "\" max=\"" + String(angleMax[i]) +
             "\" value=\"" + String(currentInt) +
             "\" oninput=\"onSliderInput(" + String(i) + ", this.value)\"" +
             " onchange=\"onSliderChange(" + String(i) + ", this.value)\">";
    html += "<input type=\"number\" id=\"num" + String(i) + "\" min=\"" + String(angleMin[i]) + "\" max=\"" + String(angleMax[i]) +
             "\" value=\"" + String(currentInt) +
             "\" onchange=\"onNumberChange(" + String(i) + ", this.value)\">";
    html += "</div></div>";
  }

  html += R"HTML(
  <script>
    var lastSent = [0,0,0,0,0];
    var THROTTLE_MS = 100;

    function onSliderInput(i, value) {
      document.getElementById('num' + i).value = value;
      var now = Date.now();
      if (now - lastSent[i] < THROTTLE_MS) return;
      lastSent[i] = now;
      sendAngle(i, value);
    }

    function onSliderChange(i, value) {
      document.getElementById('num' + i).value = value;
      lastSent[i] = Date.now();
      sendAngle(i, value);
    }

    function onNumberChange(i, value) {
      document.getElementById('slider' + i).value = value;
      lastSent[i] = Date.now();
      sendAngle(i, value);
    }

    function sendAngle(i, value) {
      fetch('/set?ch=' + i + '&angle=' + value).catch(function(err) {
        console.log('Request failed:', err);
      });
    }

    function goHome() {
      fetch('/goHome').then(function() {
        setTimeout(function() { location.reload(); }, 2000);
      }).catch(function(err) {
        alert('Failed to go home: ' + err);
      });
    }
  </script>
</body>
</html>
)HTML";

  return html;
}
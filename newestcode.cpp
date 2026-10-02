#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------------- Access point (hotspot) ----------------
const char* AP_SSID = "Base";
const char* AP_PASS = "12345678";   // at least 8 characters

// ---------------- Pins ----------------
#define SENSOR_MODE INPUT_PULLDOWN
#define CONTACT     HIGH

#define POGO_STAGE_1 15
#define POGO_STAGE_2 2
#define POGO_STAGE_3 5

#define COM_PIN_R1_1 12
#define COM_PIN_R1_2 13
#define COM_PIN_R2_1 14
#define COM_PIN_R2_2 27
#define COM_PIN_R3_1 26
#define COM_PIN_R3_2 25

#define DONE_PIN_1 18     // <- DONE output of arm 1
#define DONE_PIN_2 19     // <- DONE output of arm 2
#define DONE_PIN_3 21     // <- DONE output of arm 3

// ---------------- Timing ----------------
#define USE_DONE_WIRE 1                          // 1 = arm tells us when it is finished
const unsigned long ARM2_DELAY_MS = 2000;        // Arm 1 back home -> wait -> Arm 2 starts
const unsigned long ARM3_DELAY_MS = 2000;        // Arm 2 started   -> wait -> Arm 3 starts
const unsigned long DONE_IGNORE_MS = 200;        // ignore DONE right after a code change
const unsigned long PICK_TIME_MS = 8000;         // only if USE_DONE_WIRE = 0
const unsigned long BACK_TIME_MS = 6000;         // only if USE_DONE_WIRE = 0

const char* STATE_NAME[] = {"HOME", "PICK UP", "PUT DOWN", "BACK HOME"};

struct Arm {
    int id;
    int state;            // 0=00 HOME, 1=01 PICK UP, 2=10 PUT DOWN, 3=11 BACK HOME
    int pinStage;         // pogo pin at the put-down spot
    int com1, com2;       // code out to the arm
    int donePin;          // feedback in from the arm (HIGH = finished)
    bool lastPin;
    bool lastDone;
    unsigned long since;    // millis() when the state last changed
    unsigned long startMs;  // millis() when the arm last entered 01
    bool back;              // arm is home again (state 11 finished)
    unsigned long backMs;   // millis() when it got back home
};

Arm arms[3] = {
    {1, 0, POGO_STAGE_1, COM_PIN_R1_1, COM_PIN_R1_2, DONE_PIN_1, false, false, 0, 0, false, 0},
    {2, 0, POGO_STAGE_2, COM_PIN_R2_1, COM_PIN_R2_2, DONE_PIN_2, false, false, 0, 0, false, 0},
    {3, 0, POGO_STAGE_3, COM_PIN_R3_1, COM_PIN_R3_2, DONE_PIN_3, false, false, 0, 0, false, 0},
};

bool autoMode = true;      // false = state machine paused, control from the web page
WebServer server(80);

// ---------------- Log (Serial + web) ----------------
const int LOG_MAX = 25;
String logBuf[LOG_MAX];
int logCount = 0;

void logMsg(const String &msg) {
    String line = String(millis() / 1000.0, 1) + "s  " + msg;
    Serial.println(line);
    if (logCount < LOG_MAX) {
        logBuf[logCount++] = line;
    } else {
        for (int i = 1; i < LOG_MAX; i++) logBuf[i - 1] = logBuf[i];
        logBuf[LOG_MAX - 1] = line;
    }
}

// ---------------- Arm logic ----------------
// Noise filter only: pin must read CONTACT on several consecutive reads
bool touching(int pin) {
    for (int i = 0; i < 5; i++) {
        if (digitalRead(pin) != CONTACT) return false;
        delayMicroseconds(200);
    }
    return true;
}

void applyOutputs(Arm &a) {
    digitalWrite(a.com1, (a.state >> 1) & 1);
    digitalWrite(a.com2, a.state & 1);
}

void changeState(Arm &a, int newState, const char* reason) {
    int old = a.state;
    a.state = newState;
    a.since = millis();
    a.back  = false;
    if (newState == 1) a.startMs = millis();
    applyOutputs(a);
    char buf[140];
    snprintf(buf, sizeof(buf), "ARM %d: %d%d %s -> %d%d %s | %s",
             a.id, old >> 1, old & 1, STATE_NAME[old],
             newState >> 1, newState & 1, STATE_NAME[newState], reason);
    logMsg(buf);
}

// Has the arm finished the moves for its current code?
bool armDone(Arm &a, unsigned long fallbackMs) {
    unsigned long elapsed = millis() - a.since;
#if USE_DONE_WIRE
    (void)fallbackMs;
    return elapsed >= DONE_IGNORE_MS && digitalRead(a.donePin) == HIGH;
#else
    return elapsed >= fallbackMs;
#endif
}

bool canStart(int i) {
    if (i == 0) return true;                                   // Arm 1: at once
    if (i == 1)                                                // Arm 2: Arm 1 back + 2 s
        return arms[0].back && (millis() - arms[0].backMs >= ARM2_DELAY_MS);
    return arms[1].state >= 1 &&                               // Arm 3: Arm 2 started + 2 s
           (millis() - arms[1].startMs >= ARM3_DELAY_MS);
}

void runArm(int i) {
    Arm &a = arms[i];
    switch (a.state) {
        case 0:   // 00 HOME: wait for this arm's turn
            if (canStart(i)) {
                if (i == 0)      changeState(a, 1, "start, picking up the object");
                else if (i == 1) changeState(a, 1, "Arm 1 back home + delay, picking up the object");
                else             changeState(a, 1, "delay after Arm 2 started, picking up the object");
            }
            break;

        case 1:   // 01 PICK UP: wait until the arm has picked the object
            if (armDone(a, PICK_TIME_MS)) changeState(a, 2, "object picked, putting it down");
            break;

        case 2:   // 10 PUT DOWN: wait for the pogo pin to connect (object on stage)
            if (touching(a.pinStage)) changeState(a, 3, "pogo pin HIGH, arm going back home");
            break;

        case 3:   // 11 BACK HOME: wait until the arm is home again
            if (!a.back && armDone(a, BACK_TIME_MS)) {
                a.back   = true;
                a.backMs = millis();
                char buf[60];
                snprintf(buf, sizeof(buf), "ARM %d: back at home (original state)", a.id);
                logMsg(buf);
            }
            break;
    }
}

void resetAll() {
    for (auto &a : arms) {
        a.state = 0;
        a.since = millis();
        a.back  = false;
        applyOutputs(a);
    }
    logMsg(">>> RESET, all arms back to 00 HOME");
}

// ---------------- Web page ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Arm Controller</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;padding:14px;background:#111;color:#eee}
 h2{margin:0 0 10px}
 .card{background:#1e1e1e;border-radius:10px;padding:12px;margin-bottom:12px}
 .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
 .code{font-size:28px;font-weight:700;font-family:monospace}
 .s0{color:#888}.s1{color:#f5a623}.s2{color:#4cd964}.s3{color:#4aa3ff}
 button{background:#333;color:#eee;border:0;border-radius:8px;padding:9px 13px;font-size:15px}
 button.on{background:#0a84ff}
 button.red{background:#c0392b}
 .pin{display:inline-block;padding:2px 8px;border-radius:6px;background:#333;font-size:13px}
 .pin.hi{background:#4cd964;color:#000}
 pre{margin:0;font-size:12px;max-height:240px;overflow:auto;white-space:pre-wrap}
 small{color:#999}
</style></head><body>
<h2>Robot Arm Controller</h2>
<div class="card row">
 <button id="modeBtn" onclick="toggleMode()">...</button>
 <button class="red" onclick="cmd('/reset')">Reset all to 00</button>
 <small id="ip"></small>
</div>
<div id="arms"></div>
<div class="card"><b>Event log</b><pre id="log"></pre></div>
<script>
const names=['HOME','PICK UP','PUT DOWN','BACK HOME'];
const codes=['00','01','10','11'];
let auto=true;
function cmd(u){fetch(u).then(refresh)}
function toggleMode(){cmd('/auto?on='+(auto?0:1))}
function setState(a,s){cmd('/set?arm='+a+'&state='+s)}
function refresh(){
 fetch('/status').then(r=>r.json()).then(d=>{
  auto=d.auto;
  const mb=document.getElementById('modeBtn');
  mb.textContent=auto?'Mode: AUTO (tap for manual)':'Mode: MANUAL (tap for auto)';
  mb.className=auto?'on':'';
  document.getElementById('ip').textContent=d.ip;
  let h='';
  d.arms.forEach(a=>{
   h+='<div class="card"><div class="row"><b>Arm '+a.id+'</b>'+
      '<span class="code s'+a.state+'">'+codes[a.state]+'</span>'+
      '<span class="s'+a.state+'">'+names[a.state]+'</span>'+
      '<span class="pin '+(a.pin?'hi':'')+'">pogo '+(a.pin?'HIGH':'LOW')+'</span>'+
      '<span class="pin '+(a.done?'hi':'')+'">arm '+(a.done?'DONE':'busy')+'</span></div>'+
      '<div class="row" style="margin-top:8px">';
   for(let s=0;s<4;s++){
     h+='<button class="'+(a.state==s?'on':'')+'" onclick="setState('+a.id+','+s+')">'+codes[s]+'</button>';
   }
   h+='</div></div>';
  });
  document.getElementById('arms').innerHTML=h;
  const lg=document.getElementById('log');
  const atBottom=lg.scrollTop+lg.clientHeight>=lg.scrollHeight-20;
  lg.textContent=d.log.join('\n');
  if(atBottom)lg.scrollTop=lg.scrollHeight;
 }).catch(()=>{});
}
setInterval(refresh,400);refresh();
</script></body></html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleStatus() {
    String ip = WiFi.softAPIP().toString();
    String j = "{\"auto\":" + String(autoMode ? "true" : "false") + ",\"ip\":\"" + ip + "\",\"arms\":[";
    for (int i = 0; i < 3; i++) {
        if (i) j += ",";
        j += "{\"id\":" + String(arms[i].id) +
             ",\"state\":" + String(arms[i].state) +
             ",\"pin\":" + String(digitalRead(arms[i].pinStage) == CONTACT ? 1 : 0) +
             ",\"done\":" + String(digitalRead(arms[i].donePin) == HIGH ? 1 : 0) + "}";
    }
    j += "],\"log\":[";
    for (int i = 0; i < logCount; i++) {
        if (i) j += ",";
        j += "\"" + logBuf[i] + "\"";
    }
    j += "]}";
    server.send(200, "application/json", j);
}

void handleReset() { resetAll(); server.send(200, "text/plain", "ok"); }

void handleAuto() {
    autoMode = server.arg("on") == "1";
    logMsg(autoMode ? ">>> AUTO mode ON" : ">>> MANUAL mode (state machine paused)");
    server.send(200, "text/plain", "ok");
}

void handleSet() {
    int arm = server.arg("arm").toInt();
    int st  = server.arg("state").toInt();
    if (arm < 1 || arm > 3 || st < 0 || st > 3) { server.send(400, "text/plain", "bad"); return; }
    // Setting a state by hand switches to manual so the auto logic doesn't overwrite it
    if (autoMode) { autoMode = false; logMsg(">>> switched to MANUAL"); }
    changeState(arms[arm - 1], st, "set from web page");
    server.send(200, "text/plain", "ok");
}

// ---------------- WiFi ----------------
void startWiFi() {
    WiFi.mode(WIFI_AP);
    IPAddress apIP(192, 132, 10, 11);
    IPAddress apGateway(192, 132, 10, 11);
    IPAddress apSubnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apGateway, apSubnet);   // must be called before softAP()
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(500);
    Serial.printf("Hotspot '%s' started (password %s).\n", AP_SSID, AP_PASS);
    Serial.print("Connect to it, then open  http://");
    Serial.println(WiFi.softAPIP());   // 192.132.10.11
}

// ---------------- Setup / loop ----------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    for (auto &a : arms) {
        pinMode(a.pinStage, SENSOR_MODE);
        pinMode(a.donePin, INPUT_PULLDOWN);
        pinMode(a.com1, OUTPUT);
        pinMode(a.com2, OUTPUT);
        a.state = 0;
        a.since = millis();
        applyOutputs(a);
    }

    startWiFi();

    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.on("/reset", handleReset);
    server.on("/auto", handleAuto);
    server.on("/set", handleSet);
    server.begin();

    logMsg("Ready. Arm 1 starts now, Arm 2 starts 2 s after Arm 1 is back, Arm 3 starts 2 s after Arm 2");
    if (!USE_DONE_WIRE) logMsg("NOTE: USE_DONE_WIRE = 0, using fixed PICK/BACK times");
}

void loop() {
    server.handleClient();

    if (Serial.available() && Serial.read() == 'r') resetAll();

    if (autoMode) {
        for (int i = 0; i < 3; i++) runArm(i);
    }

    // Log pogo / DONE line changes so you can see the sensors react
    for (auto &a : arms) {
        bool p = digitalRead(a.pinStage) == CONTACT;
        if (p != a.lastPin) {
            a.lastPin = p;
            char buf[60];
            snprintf(buf, sizeof(buf), "   pogo pin %d (arm %d) -> %s", a.pinStage, a.id, p ? "HIGH" : "LOW");
            logMsg(buf);
        }
        bool d = digitalRead(a.donePin) == HIGH;
        if (d != a.lastDone) {
            a.lastDone = d;
            char buf[60];
            snprintf(buf, sizeof(buf), "   DONE pin %d (arm %d) -> %s", a.donePin, a.id, d ? "HIGH" : "LOW");
            logMsg(buf);
        }
    }
}

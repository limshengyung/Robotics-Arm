#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// --- Variables ---
float theta1;
float theta2;
float x;
float y;
float z;
float delta;
float theta3;
float psi = 180; // Desired gripper orientation.

const int wristPin = 2;
const int gripperPin = 38; // Not explicitly used in movement logic, but declared

int j1Speed = 600;
int j2Speed = 800;
int baseSpeed = 400;
int previousMillis = 0;

// --- Motor & Servo Objects ---
Servo wrist; 
AccelStepper baseStepper(1, 5, 4);   // (1 is default driver), STEP, DIR
AccelStepper j1Stepper_L(1, 7, 6);   // (1 is default driver), STEP, DIR
AccelStepper j1Stepper_R(1, 17, 15); // (1 is default driver), STEP, DIR
AccelStepper j2Stepper(1, 10, 9);    // (1 is default driver), STEP, DIR

// --- ESP-NOW Data Structure ---
typedef struct struct_message {
  float x;
  float y;
  float z;
} struct_message;

struct_message myData;

// --- ESP-NOW Callback ---
// This triggers whenever data is received wirelessly
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}

// --- Helper Function: Check Distance ---
bool isClose(AccelStepper s1, int threshold) {
  if (abs(s1.distanceToGo()) < threshold) {
    return true;
  }
  return false;
}

// --- Inverse Kinematics Core Function ---
void inverseKinematics(float targetX, float targetY, float targetZ) {
  // Reset speeds
  j1Speed = 600;
  j2Speed = 800;
  baseSpeed = 800;

  const float pi = 3.141593;
  float j1 = 255.68; // Length J1 in mm
  float j2 = 428.40; // Length J2 in mm

  // 1. Calculate Base Rotation (Z-axis)
  delta = atan(targetZ / targetX); 
  delta *= (180 / pi); // Convert Radians to degrees

  // 2. Adjust X for 3D extension
  float x2 = sqrt(sq(targetZ) + sq(targetX)) - targetX;
  if (targetZ != 0) targetX += x2;

  // 3. Calculate 2D Joint Angles (Law of Cosines)
  theta2 = -acos((sq(targetX) + sq(targetY) - sq(j1) - sq(j2)) / (2 * j1 * j2));
  theta1 = atan(targetY / targetX) + atan((j2 * sin(theta2)) / (j1 + j2 * cos(theta2)));

  // Convert Radians to degrees
  theta2 *= (180 / pi);
  theta1 *= (180 / pi);

  // 4. Quadrant Corrections (Mirrors across axis if negative)
  if (theta2 < 0 && theta1 > 0) {
    if (theta1 < 90) {
      theta1 += (180 - (theta1 * 2)); 
      theta2 *= -1;
    } else if (theta1 > 90) {
      theta1 = theta1 - (2 * (theta1 - 90)); 
      theta2 *= -1;
    }
  } else if (theta1 < 0 && theta2 < 0) {
    theta1 *= -1;
    theta2 *= -1;
  }

  // 5. Calculate End Effector / Gripper Orientation
  theta3 = psi - theta2 - (90 - theta1);
  theta3 = 180 - theta3;

  // 6. Smooth deceleration when close to target
  if (isClose(j1Stepper_L, 200)) j1Speed /= 2;
  if (isClose(j2Stepper, 200)) j2Speed /= 2;
  if (isClose(baseStepper, 200)) baseSpeed /= 2;

  // 7. Write positions to steppers (applying gear ratios)
  if (theta1 > 90) {
    j1Stepper_L.moveTo((90 - theta1) * -37.037037);
    j1Stepper_R.moveTo((90 - theta1) * 37.037037);
    j1Stepper_L.setSpeed(j1Speed);
    j1Stepper_R.setSpeed(-j1Speed);
  } else {
    j1Stepper_L.moveTo((90 - theta1) * -37.037037);
    j1Stepper_R.moveTo((90 - theta1) * 37.037037);
    j1Stepper_L.setSpeed(-j1Speed);
    j1Stepper_R.setSpeed(j1Speed);
  }

  j2Stepper.moveTo(theta2 * 51.8);
  j2Stepper.setSpeed(j2Speed);
  
  baseStepper.moveTo(delta * 16.666667);
  baseStepper.setSpeed(baseSpeed);

  // 8. Execute Movements
  if (theta3 > 0) wrist.write(theta3 * 0.666667);
  
  j1Stepper_L.runSpeedToPosition();
  j1Stepper_R.runSpeedToPosition();
  j2Stepper.runSpeedToPosition();
  baseStepper.runSpeedToPosition();
}

void setup() {
  // Init steppers (Half steps)
  baseStepper.setMaxSpeed(baseSpeed); 
  j1Stepper_L.setMaxSpeed(j1Speed); 
  j1Stepper_R.setMaxSpeed(j1Speed); 
  j2Stepper.setMaxSpeed(j2Speed); 

  wrist.attach(wristPin); 

  Serial.begin(115200);

  // --- WIFI & ESP-NOW SETUP ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // --- Start Position ---
  x = 400;
  y = 0; // Remember: in this project, down is positive!
  z = 0;
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Update coordinate targets based on wireless remote input
  // Every 20ms, it increments or decrements the target to create smooth tracking
  if (currentMillis - previousMillis >= 20) {
    previousMillis = currentMillis;
    
    // X Axis bounds checking
    if (myData.x > 400) x++;
    else if (myData.x < 100) x--;
    
    // Z Axis bounds checking (The author noted using Y from joystick to control Z for testing)
    if (myData.y > 400) z++; 
    else if (myData.y < 100) z--;
  }

  // Calculate angles and move the arm to the new (x, y, z) target
  inverseKinematics(x, y, z);
}
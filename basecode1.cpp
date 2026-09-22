#include <Arduino.h>

// --- Pogo Pin Inputs (Sensors) ---
#define POGO_BACKSTAGE_1 1  
#define POGO_BACKSTAGE_2 2  
#define POGO_BACKSTAGE_3 3  

#define POGO_STAGE_1 4      
#define POGO_STAGE_2 5      
#define POGO_STAGE_3 6      

// --- Arm Output Pins (COM) ---
#define COM_PIN_R1_1 7      
#define COM_PIN_R1_2 8      
#define COM_PIN_R2_1 9      
#define COM_PIN_R2_2 10     
#define COM_PIN_R3_1 11     
#define COM_PIN_R3_2 12     

// State tracking (0 = 00, 1 = 01, 2 = 10, 3 = 11)
int stateArm1 = 0;
int stateArm2 = 0;
int stateArm3 = 0;

// Timers to track the last time each arm changed states
unsigned long lastTimeArm1 = 0;
unsigned long lastTimeArm2 = 0;
unsigned long lastTimeArm3 = 0;

// The cooldown time in milliseconds (200ms is usually perfect for physical objects)
const unsigned long DEBOUNCE_DELAY = 200; 

// Updated function with non-blocking timer
void handleArmStateMachine(int &currentState, unsigned long &lastChangeTime, int pinBackstage, int pinStage, int comPin1, int comPin2) {
    unsigned long currentMillis = millis();
    
    // Non-blocking Debounce: Ignore sensor readings if the state just changed recently
    if (currentMillis - lastChangeTime < DEBOUNCE_DELAY) {
        return; // Exit the function early and check again next loop
    }

    switch (currentState) {
        
        // 00: Initial State
        case 0:
            digitalWrite(comPin1, LOW);
            digitalWrite(comPin2, LOW);
            
            if (digitalRead(pinBackstage) == LOW) {
                currentState = 1;
                lastChangeTime = currentMillis; // Reset the cooldown timer
            }
            break;

        // 01: Arm moving to the stage
        case 1:
            digitalWrite(comPin1, LOW);
            digitalWrite(comPin2, HIGH);
            
            if (digitalRead(pinStage) == LOW) {
                currentState = 2;
                lastChangeTime = currentMillis;
            }
            break;

        // 10: Arm staying at the stage
        case 2:
            digitalWrite(comPin1, HIGH);
            digitalWrite(comPin2, LOW);
            
            // Wait for the word to be lifted off the stage
            if (digitalRead(pinStage) == HIGH) { 
                currentState = 3;
                lastChangeTime = currentMillis;
            }
            break;

        // 11: Arm returns word to backstage
        case 3:
            digitalWrite(comPin1, HIGH);
            digitalWrite(comPin2, HIGH);
            
            if (digitalRead(pinBackstage) == LOW) {
                currentState = 0;
                lastChangeTime = currentMillis;
            }
            break;
    }
}

void setup() {
    pinMode(COM_PIN_R1_1, OUTPUT); pinMode(COM_PIN_R1_2, OUTPUT);
    pinMode(COM_PIN_R2_1, OUTPUT); pinMode(COM_PIN_R2_2, OUTPUT);
    pinMode(COM_PIN_R3_1, OUTPUT); pinMode(COM_PIN_R3_2, OUTPUT);

    pinMode(POGO_BACKSTAGE_1, INPUT_PULLUP);
    pinMode(POGO_BACKSTAGE_2, INPUT_PULLUP);
    pinMode(POGO_BACKSTAGE_3, INPUT_PULLUP);
    pinMode(POGO_STAGE_1, INPUT_PULLUP);
    pinMode(POGO_STAGE_2, INPUT_PULLUP);
    pinMode(POGO_STAGE_3, INPUT_PULLUP);

    digitalWrite(COM_PIN_R1_1, LOW); digitalWrite(COM_PIN_R1_2, LOW);
    digitalWrite(COM_PIN_R2_1, LOW); digitalWrite(COM_PIN_R2_2, LOW);
    digitalWrite(COM_PIN_R3_1, LOW); digitalWrite(COM_PIN_R3_2, LOW);
}

void loop() {
    // We now pass the specific timer variable for each arm so they debounce independently
    handleArmStateMachine(stateArm1, lastTimeArm1, POGO_BACKSTAGE_1, POGO_STAGE_1, COM_PIN_R1_1, COM_PIN_R1_2);
    handleArmStateMachine(stateArm2, lastTimeArm2, POGO_BACKSTAGE_2, POGO_STAGE_2, COM_PIN_R2_1, COM_PIN_R2_2);
    handleArmStateMachine(stateArm3, lastTimeArm3, POGO_BACKSTAGE_3, POGO_STAGE_3, COM_PIN_R3_1, COM_PIN_R3_2);
}

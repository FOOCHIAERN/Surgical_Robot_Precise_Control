#include <AccelStepper.h>

// --- PIN DEFINITIONS ---
#define M1_DIR_PIN 8
#define M1_STEP_PIN 9
#define M1_EN_PIN 10

#define M2_DIR_PIN 5
#define M2_STEP_PIN 6
#define M2_EN_PIN 7

#define M3_DIR_PIN 2
#define M3_STEP_PIN 3
#define M3_EN_PIN 4

// --- MOTOR PARAMETERS ---
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPPING = 16;
const float RATIO_BASE = 27.0;
const float RATIO_SHOULDER = 19.0;
const float RATIO_ELBOW = 14.0;

// Calculation: (200 * 16 * 27) = 86,400 steps per full output revolution
// 60 degrees is 1/6th of a revolution: 86,400 / 6 = 14,400 steps
const long TARGET_BASE = (MOTOR_STEPS_PER_REV * MICROSTEPPING * RATIO_BASE) * 6;
const long TARGET_SHOULDER = (MOTOR_STEPS_PER_REV * MICROSTEPPING * RATIO_SHOULDER) * 6;
const long TARGET_ELBOW = (MOTOR_STEPS_PER_REV * MICROSTEPPING * RATIO_ELBOW) * 6;


// Initialize the stepper (1 = Driver with Step/Dir pins)
AccelStepper joint1(1, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper joint2(1, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper joint3(1, M3_STEP_PIN, M3_DIR_PIN);

void setup() {
  Serial.begin(115200);

  Serial.println("Starting 3DoF 60-Degree Test...");
  // Enable the driver (Active LOW)
  pinMode(M1_EN_PIN, OUTPUT); 
  pinMode(M2_EN_PIN, OUTPUT); 
  pinMode(M3_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  digitalWrite(M3_EN_PIN, LOW);

  // Set up Base (Joint 1)
  //joint1.setMaxSpeed(6000.0);
  joint1.setMaxSpeed(600.0);
  joint1.setAcceleration(200.0); 

  // Set up Shoulder (Joint 2)
  //joint2.setMaxSpeed(2000.0);
  joint2.setMaxSpeed(400.0);
  joint2.setAcceleration(200.0);

  // Set up Elbow (Joint 3)
  //joint3.setMaxSpeed(2000.0);
  joint3.setMaxSpeed(200.0);
  joint3.setAcceleration(100.0);
}


void loop() {
  // If reached target, move back to 0 (60-degree oscillation)
  // --- JOINT 1 CONTROL ---
  if (joint1.distanceToGo() == 0) {
    if (joint1.currentPosition() == 0) {
      joint1.moveTo(TARGET_BASE);
    } else {
      joint1.moveTo(0);
    }
  }

  // --- JOINT 2 CONTROL ---
  if (joint2.distanceToGo() == 0) {
    if (joint2.currentPosition() == 0) {
      joint2.moveTo(TARGET_SHOULDER);
    } else {
      joint2.moveTo(0);
    }
  }

  // --- JOINT 3 CONTROL ---
  if (joint3.distanceToGo() == 0) {
    if (joint3.currentPosition() == 0) {
      joint3.moveTo(-TARGET_ELBOW);
    } else {
      joint3.moveTo(0);
    }
  }

  // Must run constantly
  joint1.run();
  joint2.run();
  joint3.run();
}

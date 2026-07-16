#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

// ==================== MOTOR DRIVER PINS ====================
#define AIN1 3
#define AIN2 4
#define PWMA 5
#define BIN1 7
#define BIN2 8
#define PWMB 6
#define STBY 9

// ==================== ULTRASONIC PINS ====================
#define TRIG_PIN 18
#define ECHO_PIN 19

// ==================== STEPPER PINS ====================
#define STEP_PIN 13
#define DIR_PIN 12

// ==================== SERVO PIN ====================
#define SERVO_PIN 44

// ==================== BUTTONS ====================
int buttoncalibrate = 11;
int buttonstart = 10;

// ==================== MOTOR DRIVER SETUP ====================
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ==================== QTR SENSOR ====================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ==================== COLOR SENSOR ====================
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ==================== SERVO ====================
Servo palletServo;
#define SERVO_STOP 1500
#define SERVO_CW   1000
#define SERVO_CCW  2000
#define SERVO_TIME 1700

// ==================== STEPPER ====================
#define STEP_DELAY 350
#define STEP_CHECK 50
const int STEPS_PER_REVOLUTION = 200;

// ==================== PID ====================
float Kp = 0.3;
float Kd = 1.5;
int basespeed = 70;
int turnspeed = 30;
int maxspeed = 80;
int lastError = 0;
int lastDirection = 1;

// ==================== NAVIGATION STATE ====================
int intersectionCount = 0;
boolean atIntersectionFlag = false;
boolean navigationDone = false;
boolean isTurning = false;
boolean lineFollowingActive = true;
boolean rackReached = false;

// ==================== PALLET STATE ====================
boolean blueDetected = false;
boolean finalRotationComplete = false;
boolean loweringComplete = false;
boolean palletCycleComplete = false;
int stepsInFinalRotation = 0;
int stepsInLowering = 0;
int stepCounter = 0;

void setup() {
  Serial.begin(9600);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Stepper
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // LOW = UP

  // Servo
  palletServo.attach(SERVO_PIN);
  palletServo.writeMicroseconds(SERVO_STOP);

  // QTR
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  // Color sensor
  if (tcs.begin()) {
    Serial.println("TCS34725 ready");
  } else {
    Serial.println("TCS34725 not found!");
    while (1);
  }

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT);

  // Calibrate
  while (digitalRead(buttoncalibrate) == LOW) {}
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  brake(motor1, motor2);

  // Wait for start
  while (digitalRead(buttonstart) == LOW) {}
  delay(1000);
  Serial.println("Starting!");
}

// ==================== ULTRASONIC ====================
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.0343 / 2;
}

// ==================== LINE FOLLOWING ====================
boolean atIntersection() {
  return (sensorValues[0] > 700 && sensorValues[1] > 700 &&
          sensorValues[6] > 700 && sensorValues[7] > 700);
}

boolean lineLost() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 600) return false;
  }
  return true;
}

boolean approachingTurn() {
  boolean leftEdge = (sensorValues[0] > 600 || sensorValues[1] > 600);
  boolean rightEdge = (sensorValues[6] > 600 || sensorValues[7] > 600);
  boolean middleClear = (sensorValues[3] < 300 && sensorValues[4] < 300);
  return (leftEdge || rightEdge) && middleClear;
}

void followLine() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;

  if (error < -500) lastDirection = -1;
  if (error > 500) lastDirection = 1;

  int D = error - lastError;
  lastError = error;

  int motorspeed = (error * Kp) + (D * Kd);
  int currentBasespeed = approachingTurn() ? turnspeed : basespeed;

  int motorspeeda = currentBasespeed + motorspeed;
  int motorspeedb = currentBasespeed - motorspeed;

  if (motorspeeda > maxspeed) motorspeeda = maxspeed;
  if (motorspeedb > maxspeed) motorspeedb = maxspeed;
  if (motorspeeda < 0) motorspeeda = 0;
  if (motorspeedb < 0) motorspeedb = 0;

  motor1.drive(motorspeeda);
  motor2.drive(motorspeedb);
}

void recoverLine() {
  if (lastDirection == 1) {
    motor1.drive(40);
    motor2.drive(-40);
  } else {
    motor1.drive(-40);
    motor2.drive(40);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > 600 || sensorValues[4] > 600) {
      brake(motor1, motor2);
      delay(50);
      return;
    }
  }
  brake(motor1, motor2);
}

// ==================== TURNS ====================
void turnRight() {
  Serial.println("Turning right");
  isTurning = true;
  lineFollowingActive = false;
  brake(motor1, motor2);
  delay(200);
  motor1.drive(-80);
  motor2.drive(80);
  delay(500);
  brake(motor1, motor2);
  delay(200);
  isTurning = false;
  lineFollowingActive = true;
}

void turnLeft() {
  Serial.println("Turning left");
  isTurning = true;
  lineFollowingActive = false;
  brake(motor1, motor2);
  delay(200);
  motor1.drive(80);
  motor2.drive(-80);
  delay(600);
  brake(motor1, motor2);
  delay(200);
  isTurning = false;
  lineFollowingActive = true;
}

void goStraight() {
  motor1.drive(50);
  motor2.drive(50);
  delay(350);
  brake(motor1, motor2);
  delay(100);
}

// ==================== INTERSECTION HANDLING ====================
void handleIntersection() {
  intersectionCount++;
  Serial.print("Intersection: ");
  Serial.println(intersectionCount);

  if (intersectionCount == 1 || intersectionCount == 2) {
    goStraight();
  } else if (intersectionCount == 3) {
    turnRight();
  } else if (intersectionCount == 4) {
    goStraight();
  } else if (intersectionCount == 5) {
    turnLeft();
    navigationDone = true;
    Serial.println("Navigation done — approaching rack");
  }
}

// ==================== PALLET CYCLE ====================
void runPalletCycle() {
  Serial.println("Starting pallet cycle");

  // Phase 1 + 2: Move up until blue detected then 1 extra rotation
  digitalWrite(DIR_PIN, LOW); // UP
  blueDetected = false;
  finalRotationComplete = false;
  stepsInFinalRotation = 0;
  stepCounter = 0;

  while (!finalRotationComplete) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);

    if (!blueDetected) {
      stepCounter++;
      if (stepCounter >= STEP_CHECK) {
        stepCounter = 0;
        float r, g, b;
        tcs.getRGB(&r, &g, &b);
        if (b > r && b > g) {
          Serial.println("Blue detected!");
          blueDetected = true;
          stepsInFinalRotation = 0;
        }
      }
    } else {
      stepsInFinalRotation++;
      if (stepsInFinalRotation >= STEPS_PER_REVOLUTION) {
        finalRotationComplete = true;
      }
    }
  }

  Serial.println("At target slot");

  // Push pallet
  Serial.println("Pushing pallet");
  palletServo.writeMicroseconds(SERVO_CW);
  delay(SERVO_TIME);
  palletServo.writeMicroseconds(SERVO_STOP);

  delay(500);

  // Retract
  Serial.println("Retracting");
  palletServo.writeMicroseconds(SERVO_CCW);
  delay(SERVO_TIME);
  palletServo.writeMicroseconds(SERVO_STOP);

  Serial.println("Pallet placed");

  // Phase 3: Lower elevator
  Serial.println("Lowering elevator");
  digitalWrite(DIR_PIN, HIGH); // DOWN
  stepsInLowering = 0;
  while (stepsInLowering < 5500) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
    stepsInLowering++;
  }

  Serial.println("Elevator lowered");
  palletCycleComplete = true;
}

// ==================== MAIN LOOP ====================
void loop() {
  // Phase 4 — everything done
  if (palletCycleComplete) {
    brake(motor1, motor2);
    return;
  }

  // Phase 3 — rack reached, run pallet cycle
  if (rackReached) {
    runPalletCycle();
    return;
  }

  // Phase 2 — navigation done, approach rack at full speed
  if (navigationDone) {
    float distance = getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance > 0 && distance <= 2.0) {
      Serial.println("Rack reached — stopping");
      brake(motor1, motor2);
      delay(200);
      rackReached = true;
      return;
    }

    // Follow line at full speed toward rack
    qtr.readLineBlack(sensorValues);
    if (lineLost()) {
      recoverLine();
      return;
    }
    followLine();
    return;
  }

  // Phase 1 — grid navigation
  if (!isTurning && lineFollowingActive) {
    qtr.readLineBlack(sensorValues);

    boolean intersection = atIntersection();
    if (intersection && !atIntersectionFlag) {
      atIntersectionFlag = true;
      handleIntersection();
      return;
    } else if (!intersection) {
      atIntersectionFlag = false;
    }

    if (lineLost()) {
      recoverLine();
      return;
    }

    followLine();
  }
}
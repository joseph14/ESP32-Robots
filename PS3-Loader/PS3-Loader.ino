#include <Arduino.h>
#include <Ps3Controller.h>
#include "../Shared/Controls.h"
#include "Wrappers.h"

unsigned long LOOP_START_NOW;

#define RIGHT_MOTOR0 25
#define RIGHT_MOTOR1 26

#define LEFT_MOTOR0 33
#define LEFT_MOTOR1 32

#define ARM_MOTOR0 21
#define ARM_MOTOR1 19

#define BUCKET_SERVO_PIN 23
#define CLAW_SERVO_PIN 22

#define AUX_LIGHTS0 18
#define AUX_LIGHTS1 5

static const int CH_NULL = -1;

static const int STICK_ACTIVATE = 115;
static const int STICK_DEADBAND = 30;

static const uint32_t SERVO_STEP_MS = 20;

static bool auxLightsOn = false;
static uint32_t lastServoStepMs = 0;
RobotControls::Controls controller;

MotorWrapper rightMotor("Right Drive Motor", RIGHT_MOTOR0, RIGHT_MOTOR1, CH_NULL, 15, true);
MotorWrapper leftMotor("Left Drive Motor", LEFT_MOTOR0, LEFT_MOTOR1, CH_NULL, 15, true);
MotorWrapper armMotor("Arm Motor", ARM_MOTOR0, ARM_MOTOR1, CH_NULL, 15, true);

ServoWrapper bucketServo("Bucket Servo", BUCKET_SERVO_PIN, 140, 10, 170, true);
ServoWrapper clawServo("Claw Servo", CLAW_SERVO_PIN, 150, 10, 170, true);

LedWrapper auxLightA("Aux Light A", AUX_LIGHTS0, CH_NULL, true);
LedWrapper auxLightB("Aux Light B", AUX_LIGHTS1, CH_NULL, true);

static inline void stopDrive() {
  rightMotor.off();
  leftMotor.off();
}

static inline void toggleAuxLights() {
  auxLightsOn = !auxLightsOn;
  auxLightA.set(auxLightsOn);
  auxLightB.off();
}

static void processDriveStick(int lx, int ly) {
  if (lx > STICK_ACTIVATE) {
    rightMotor.forward(0);
    leftMotor.reverse(0);
    return;
  }
  if (lx < -STICK_ACTIVATE) {
    rightMotor.reverse(0);
    leftMotor.forward(0);
    return;
  }
  if (ly > STICK_ACTIVATE) {
    rightMotor.forward(0);
    leftMotor.forward(0);
    return;
  }
  if (ly < -STICK_ACTIVATE) {
    rightMotor.reverse(0);
    leftMotor.reverse(0);
    return;
  }

  if (abs(lx) < STICK_DEADBAND && abs(ly) < STICK_DEADBAND) {
    stopDrive();
  }
}

static void processArmStick(int ry) {
  if (ry > STICK_ACTIVATE) {
    armMotor.forward(0);
  } else if (ry < -STICK_ACTIVATE) {
    armMotor.reverse(0);
  } else if (abs(ry) < STICK_DEADBAND) {
    armMotor.off();
  }
}

static RobotControls::ControlsInput readPs3Controls() {
  RobotControls::ControlsInput in;
  in.axisX = RobotControls::NormalizeAxis(Ps3.data.analog.stick.lx, 128.0f);
  in.axisY = RobotControls::NormalizeAxis(Ps3.data.analog.stick.ly, 128.0f);
  in.axisRX = RobotControls::NormalizeAxis(Ps3.data.analog.stick.rx, 128.0f);
  in.axisRY = RobotControls::NormalizeAxis(Ps3.data.analog.stick.ry, 128.0f);

  in.leftBumper = Ps3.data.button.l1;
  in.rightBumper = Ps3.data.button.r1;
  in.leftTrigger = Ps3.data.button.l2;
  in.rightTrigger = Ps3.data.button.r2;
  in.thumbL = Ps3.data.button.l3;
  in.thumbR = Ps3.data.button.r3;

  in.a = Ps3.data.button.cross;
  in.b = Ps3.data.button.circle;
  in.x = Ps3.data.button.square;
  in.y = Ps3.data.button.triangle;

  if (Ps3.data.button.up) in.dpad = 1;
  else if (Ps3.data.button.down) in.dpad = 2;
  else in.dpad = 0;

  in.logChanges = false;
  return in;
}

void onConnect() {
  Serial.println("PS3 Connected.");
}

static void updateServos() {
  if (LOOP_START_NOW - lastServoStepMs < SERVO_STEP_MS) return;
  lastServoStepMs = LOOP_START_NOW;

  if (controller.leftBumper && !controller.rightBumper) {
    clawServo.moveTo(clawServo.Position() + 1);
  } else if (controller.rightBumper && !controller.leftBumper) {
    clawServo.moveTo(clawServo.Position() - 1);
  }

  if (controller.rightTrigger && !controller.leftTrigger) {
    bucketServo.moveTo(bucketServo.Position() + 1);
  } else if (controller.leftTrigger && !controller.rightTrigger) {
    bucketServo.moveTo(bucketServo.Position() - 1);
  }
}

void setup() {
  Serial.begin(115200);

  rightMotor.begin();
  leftMotor.begin();
  armMotor.begin();

  auxLightA.begin(false);
  auxLightB.begin(false);

  bucketServo.begin();
  clawServo.begin();

  Ps3.attachOnConnect(onConnect);
  Ps3.begin("a0:5a:5a:a0:0f:98");

  Serial.println("PS3-Loader Ready.");
}

void loop() {
  LOOP_START_NOW = millis();

  if (!Ps3.isConnected()) {
    stopDrive();
    armMotor.off();
    delay(25);
    return;
  }

  controller.updateFrom(readPs3Controls());
  processDriveStick(Ps3.data.analog.stick.lx, Ps3.data.analog.stick.ly);
  processArmStick(Ps3.data.analog.stick.ry);
  if (controller.thumbR.rising()) toggleAuxLights();
  updateServos();
  delay(10);
}

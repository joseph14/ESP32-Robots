#include <Arduino.h>
#include <limits.h>
#include <AccelStepper.h>
#include "StepperWebServer.h"

unsigned long LOOP_START_NOW;
unsigned long lastInputDelta;
float lastInputDeltaSeconds;
unsigned long lastInputTime;

const float STEPS_PER_REV = 4096.0f;

#define STEP1_PIN 32
#define STEP2_PIN 33
#define STEP3_PIN 25
#define STEP4_PIN 26

// Try this order first for 28BYJ-48 + ULN2003
AccelStepper stepper(AccelStepper::HALF4WIRE, STEP1_PIN, STEP3_PIN, STEP2_PIN, STEP4_PIN);

StepperWebServer webUi;

const unsigned long INPUT_TIMEOUT = 250;
const unsigned long MOTOR_TIMEOUT = 500000;
bool motorsActive = false;

float commandedAngleDeg = 0.0f;
float directionSign = 1.0f;
static long lastTarget = LONG_MIN;

static long angleToSteps(float angleDeg) {
  return lroundf(angleDeg * STEPS_PER_REV / 360.0f);
}

static void rebuildStepperTarget() {
  long target = angleToSteps(commandedAngleDeg);

  if (target != lastTarget) {
    long current = stepper.currentPosition();
    long delta = target - current;

    Serial.printf(
      "[Main] Apply target: angle=%.2f targetSteps=%ld currentSteps=%ld delta=%ld dir=%.0f\n",
      commandedAngleDeg,
      target,
      current,
      delta,
      directionSign
    );

    stepper.moveTo(target);
    lastTarget = target;
    lastInputTime = LOOP_START_NOW;
    motorsActive = true;
  } else {
    Serial.printf("[Main] Target unchanged: %ld\n", target);
  }
}

void applyWebCommands() {
  static float lastAppliedSpeedStepsPerSec = -1.0f;

  float requestedSpeed = webUi.currentSpeedStepsPerSec();
  if (requestedSpeed < 1.0f) {
    requestedSpeed = 1.0f;
  }

  if (fabsf(requestedSpeed - lastAppliedSpeedStepsPerSec) > 0.5f) {
    stepper.setMaxSpeed(requestedSpeed);
    lastAppliedSpeedStepsPerSec = requestedSpeed;

    Serial.printf("[Main] Apply speed: %.2f steps/sec\n", requestedSpeed);
  }

  if (webUi.consumeTargetChanged()) {
    commandedAngleDeg = webUi.angleDeg();
    directionSign = webUi.direction();
    rebuildStepperTarget();
  }

  if (webUi.consumeStopRequested()) {
    Serial.println("[Main] Web stop requested");
    stepper.stop();
    lastInputTime = LOOP_START_NOW;
  }
}

void setup() {
  Serial.begin(115200);

  LOOP_START_NOW = millis();
  lastInputTime = LOOP_START_NOW;

  webUi.setDebug(true);
  webUi.setStepsPerRev(STEPS_PER_REV);
  webUi.setBaseSpeedStepsPerSec(800.0f);
  webUi.setSpeedMultiplier(1.0f);
  webUi.setDirection(1.0f);
  webUi.beginAP("StepperControl", "12345678");

  stepper.setMaxSpeed(400.0f);      // tune
  stepper.setAcceleration(4000.0f);  // tune

  Serial.printf("[Main] AccelStepper ready. stepsPerRev=%.1f\n", STEPS_PER_REV);
}

void loop() {
  LOOP_START_NOW = millis();
  lastInputDelta = LOOP_START_NOW - lastInputTime;
  lastInputDeltaSeconds = (float)lastInputDelta / 1000.0f;

  webUi.loop();
  applyWebCommands();

  stepper.run();

  if (motorsActive && stepper.distanceToGo() == 0) {
    motorsActive = false;
    Serial.printf("[Main] Move complete at currentSteps=%ld\n", stepper.currentPosition());
  }

  if (motorsActive) {
    if (lastInputDelta > INPUT_TIMEOUT + MOTOR_TIMEOUT) {
      stepper.stop();
      motorsActive = false;
      Serial.println("[Failsafe] Motor timeout reached, stopping stepper");
    }
  } else {
    vTaskDelay(1);
  }
}
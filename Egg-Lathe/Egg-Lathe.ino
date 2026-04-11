#include <Arduino.h>
#include <limits.h>
#include "StepperWebServer.h"
#include "TimerStepper4Coil.h"

unsigned long LOOP_START_NOW;
unsigned long lastInputDelta;
float lastInputDeltaSeconds;
unsigned long lastInputTime;

const float STEPS_PER_REV = 4096.0f;

#define STEP1_PIN 32
#define STEP2_PIN 33
#define STEP3_PIN 25
#define STEP4_PIN 26

TimerStepper4Coil stepper(
  STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN,
  TimerStepper4Coil::StepMode::Half,
  false
);

StepperWebServer webUi;

const unsigned long INPUT_TIMEOUT = 250;
const unsigned long MOTOR_TIMEOUT = 500000;
bool motorsActive = false;

static long lastTarget = LONG_MIN;

void applyWebCommands() {
  float speedNow = webUi.currentSpeedStepsPerSec();
  stepper.setSpeedStepsPerSec(speedNow);

  if (webUi.consumeTargetChanged()) {
    long target = webUi.targetSteps();
    if (target != lastTarget) {
      Serial.printf(
        "[Main] New web target: angle=%.2f targetSteps=%ld dir=%.0f speed=%.2f\n",
        webUi.angleDeg(),
        target,
        webUi.direction(),
        speedNow
      );

      stepper.moveTo(target);
      lastTarget = target;
      lastInputTime = LOOP_START_NOW;
      motorsActive = true;
    } else {
      Serial.printf("[Main] TargetChanged consumed, but target unchanged: %ld\n", target);
    }
  }

  if (webUi.consumeStopRequested()) {
    Serial.println("[Main] Web stop requested");
    stepper.stop();
    motorsActive = false;
    lastInputTime = LOOP_START_NOW;
  }
}

void setup() {
  Serial.begin(115200);

  LOOP_START_NOW = millis();
  lastInputTime = LOOP_START_NOW;

  stepper.begin(true);

  webUi.setDebug(true);
  webUi.setStepsPerRev(STEPS_PER_REV);
  webUi.setBaseSpeedStepsPerSec(800.0f);
  webUi.setSpeedMultiplier(1.0f);
  webUi.setDirection(1.0f);
  webUi.beginAP("StepperControl", "12345678");
}

void loop() {
  LOOP_START_NOW = millis();
  lastInputDelta = LOOP_START_NOW - lastInputTime;
  lastInputDeltaSeconds = (float)lastInputDelta / 1000.0f;

  webUi.loop();
  applyWebCommands();

  if (motorsActive && !stepper.isMoving()) {
    motorsActive = false;
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
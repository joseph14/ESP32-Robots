#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include "Controls.h"
#include "Wrappers.h"

unsigned long LOOP_START_NOW;
unsigned long lastInputDelta;
unsigned long lastInputTime;

using RobotControls::Controls;
Controls controller;



#define RIGHT_MOTOR0 25
#define RIGHT_MOTOR1 26

#define LEFT_MOTOR0 32
#define LEFT_MOTOR1 33

#define ARM_MOTOR0 21
#define ARM_MOTOR1 19

#define BUCKET_SERVO_PIN 23
#define CLAW_SERVO_PIN 22

#define AUX_LIGHTS0 18
#define AUX_LIGHTS1 5

static const int CH_MTR1 = 8;
static const int CH_MTR2 = 9;
static const int CH_MTR3 = 10;
static const int CH_LED1 = 12;
static const int CH_LED2 = 13;

static const int CH_NULL = -1;

static const uint32_t SERVO_STEP_MS = 20;

static bool auxLightsOn = false;
static uint32_t lastServoStepMs = 0;

MotorWrapper rightMotor("Right Drive Motor", RIGHT_MOTOR0, RIGHT_MOTOR1, CH_MTR1, 0.05f, true);
MotorWrapper leftMotor("Left Drive Motor", LEFT_MOTOR0, LEFT_MOTOR1, CH_MTR2, 0.05f, true);
MotorWrapper armMotor("Arm Motor", ARM_MOTOR0, ARM_MOTOR1, CH_MTR3, 0.05f, true);

ServoWrapper bucketServo("Bucket Servo", BUCKET_SERVO_PIN, 140, 10, 170, true);
ServoWrapper clawServo("Claw Servo", CLAW_SERVO_PIN, 150, 10, 170, true);

LedWrapper auxLightA("Aux Light A", AUX_LIGHTS0, CH_LED1, true);
LedWrapper auxLightB("Aux Light B", AUX_LIGHTS1, CH_LED2, true);

static inline void toggleAuxLights() {
  auxLightsOn = !auxLightsOn;
  auxLightA.set(auxLightsOn);
  auxLightB.off();
}


// axisValue is assumed centered at 0 (negative = left, positive = right)
// TUNE THESE:
static constexpr float EXPO   = 0.45f;     // 0 = linear, 0.3-0.7 typical, higher = softer center

static float applyExpo(float x, float expo)
{
  // x in [-1..1]
  // mix linear and cubic: y = (1-expo)*x + expo*x^3
  return (1.0f - expo) * x + expo * x * x * x;
}

int sign(float x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 1;
}



static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float lerpf(float a, float b, float t) {
  t = clampf(t, 0.0f, 1.0f);
  return a + (b - a) * t;
}




static inline int signf0(float x) {
  if (x > 0.0f) return 1;
  if (x < 0.0f) return -1;
  return 0;
}

static constexpr float HUGE_R = 1000000.0f;

// Excel R1:
// =LET(angle,a*90,_R45,6,HUGE_R,1000000,mag,ABS(angle),rad,RADIANS(mag),t,TAN(rad),k,_R45*TAN(RADIANS(45)),base,IF(mag<0.01,HUGE_R,MAX(_R45,k/t)),SIGN(angle)*base)
float radiusFromLinearBranch(float aNorm) {
  float angle = aNorm * 90.0f;   // same as [@a]*90
  const float R45 = 5.0f;

  float mag = fabsf(angle);
  float rad = mag * (float)M_PI / 180.0f;
  float t = tanf(rad);
  float k = R45 * tanf(45.0f * (float)M_PI / 180.0f);  // effectively R45

  float base;
  if (mag < 0.01f) {
    base = HUGE_R;
  } else {
    base = fmaxf(R45, k / t);
  }

  return (float)signf0(angle) * base;
}

// Excel R2:
// =LET(angle,a*90,_R45,6,RMIN,0,mag,ABS(angle),u,(mag-45)/45,uClamped,MAX(0,MIN(1,u)),base,_R45+(RMIN-_R45)*uClamped,SIGN(angle)*base)
float radiusFromDirectBranch(float aNorm) {
  float angle = aNorm * 90.0f;   // same as [@a]*90
  const float R45 = 5.0f;
  const float RMIN = 0.00f;

  float mag = fabsf(angle);
  float u = (mag - 45.0f) / 45.0f;
  float uClamped = clampf(u, 0.0f, 1.0f);
  float base = R45 + (RMIN - R45) * uClamped;

  return (float)signf0(angle) * base;
}

// Excel final:
// =IF(ABS(a)>0.5,R2,R1)
float controllerToRadius(float aNorm) {
  aNorm *= -0.999f;
  if (fabsf(aNorm) > 0.5f) {
    return radiusFromDirectBranch(aNorm);
  }
  return radiusFromLinearBranch(aNorm);
}

  

void processDriveStick(float x,float y){

  float magnitude = hypotf(x, y);

  magnitude *= clampf(fabs(y)*5,0,1);

  float adjustedThrottleValue = -magnitude*sign(y);

     //steering
    float a = x;   // normalized steering input in [-1..1]
    float R = controllerToRadius(a);

    float Rleft  = R-1;
    float Rright = R+1;


    if (fabs(Rleft)>fabs(Rright))
    {
      rightMotor.movePWM(adjustedThrottleValue*Rright/Rleft, 50);
      leftMotor.movePWM(adjustedThrottleValue, 50);
    }
    else
    {
      rightMotor.movePWM(adjustedThrottleValue, 50);
      leftMotor.movePWM(adjustedThrottleValue*Rleft/Rright, 50);
    }

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


void processTool()
{

  float lift = controller.axisRY;
 // float tilt = controller.axisRX;

  armMotor.movePWM(lift,50);
  //tiltMotor.movePWM(-tilt,50);


  //if (controller.rightTrigger) attachmentMotor.forward(50);
 // else if (controller.leftTrigger) attachmentMotor.reverse(50);


  //if (controller.x.rising()) {
   // attachmentServo.moveTo(attachmentServo.Position() < 60 ? 115 : 10);
  //}


}



void processGamepad() {

  processDriveStick(controller.axisX, controller.axisY);
  processTool();
  if (controller.thumbR.rising()) toggleAuxLights();

}


const unsigned long INPUT_TIMEOUT = 250;  // ms — adjust if needed
const unsigned long MOTOR_TIMEOUT = 5000;  // ms — adjust if needed
const unsigned long LED_TIMEOUT = 10000;  // ms — adjust if needed
const unsigned long SERVO_TIMEOUT = 15000;  // ms — adjust if needed
bool motorsActive;
bool ledsActive;
bool servoActive;


void setup() {
  Serial.begin(115200);

  rightMotor.begin();
  leftMotor.begin();
  armMotor.begin();
  motorsActive = true;

  auxLightA.begin(false);
  auxLightB.begin(false);
  ledsActive = true;

  bucketServo.begin();
  clawServo.begin();
  servoActive=true;

  RobotControls::connectController();

  lastInputTime = LOOP_START_NOW;  // initialize failsafe timer
}

void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  LOOP_START_NOW = millis();
  lastInputDelta = LOOP_START_NOW - lastInputTime;

  if (RobotControls::processControllers(controller,false)) {
    lastInputTime = LOOP_START_NOW;
    motorsActive = true;
    ledsActive = true;
    servoActive = true;
  }
  else 
  { 
    vTaskDelay(1); 
  }


 processGamepad();


  if (motorsActive)  
  {
      // Failsafe check: if no input for too long, stop motors
    if (lastInputDelta > INPUT_TIMEOUT + MOTOR_TIMEOUT) 
    {
      rightMotor.off();
      leftMotor.off();
      armMotor.off();
      motorsActive=false;
    }
    else
    {
      rightMotor.update();
      leftMotor.update();
      armMotor.update();
    }
  }

    
  if (servoActive)  
  {
      // Failsafe check: if no input for too long, stop servos
    if (lastInputDelta > INPUT_TIMEOUT + SERVO_TIMEOUT) 
    {
      bucketServo.detach();
      clawServo.detach();
      servoActive=false;
    }
    else
    {
      //servo updates
      bucketServo.update();
      clawServo.update();

    }
  }

  if (ledsActive)  
  {
      // Failsafe check: if no input for too long, stop servos
    if (lastInputDelta > INPUT_TIMEOUT + LED_TIMEOUT) 
    {
      auxLightA.off();
      auxLightB.off();
      ledsActive=false;
    }
    else
    {
      //led updates

      //float throttle = rearLeftMotor.normalizedSpeed();
      //float steering = rearLeftServo.normalizedPosition();
      //updateLeds(throttle,steering); 
    }
  }


}

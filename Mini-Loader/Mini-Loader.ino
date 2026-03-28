#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>
#include "../Shared/Controls.h"
#define BP32_MAX_GAMEPADS 1
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
unsigned long LOOP_START_NOW;
unsigned long lastInputDelta;
unsigned long lastInputTime;

enum class MacroType : uint8_t { None=0, RotateLeftStep, RotateRightStep,StepLeft,StepRight};

struct MacroRunner {
  MacroType active = MacroType::None;
  uint8_t phase = 0;
  uint32_t t0 = 0;

  bool isActive() const { return active != MacroType::None; }

  void start(MacroType m) {
    active = m;
    phase = 0;
    t0 = LOOP_START_NOW;
  }

  void stop() {
    active = MacroType::None;
    phase = 0;
  }
};

static MacroRunner gMacro;

static const uint32_t PAIR_WINDOW_MS = 20000;
static uint32_t bootMs = 0;



#include "Wrappers.h"

#define LT1 15
#define LT2 27
#define LT3 14

#define steeringServoPin 23
#define attachmentServoPin 22

#define LT_MTR0 4    // \ Used for controlling front drive motor movement
#define LT_MTR1 2    // /
#define RT_MTR0 12  // \ Used for controlling rear drive motor movement.
#define RT_MTR1 13  // /

#define LBL_MTR0 17   
#define LBL_MTR1 16   
#define RBL_MTR0 18  
#define RBL_MTR1 5    
#define BLD_T0 26  
#define BLD_T1 25  
#define RPR0 32      
#define RPR1 33     


// Pick unique LEDC channels (8..15) // leave 0-5 for servos 

static const int CH_LBL = 8;
static const int CH_RBL = 9;
static const int CH_MTR = 10;

static const int CH_LED1 = 12;
static const int CH_LED2 = 13;
static const int CH_LED3 = 14;

static const int CH_NULL = -1;




// Create motors
MotorWrapper rearDriveMotor(  "Rear Drive Motor" ,LT_MTR1 , LT_MTR0 ,CH_MTR ,15,true);
MotorWrapper frontDriveMotor( "Front Drive Motor",RT_MTR1 , RT_MTR0 ,CH_MTR ,15,true);
MotorWrapper liftMotor(       "Lift Motor"       ,LBL_MTR0, LBL_MTR1,CH_LBL ,15,true);
MotorWrapper tiltMotor(       "Tilt Motor"       ,RBL_MTR0, RBL_MTR1,CH_RBL ,15,true);
MotorWrapper attachmentMotor( "Attachment Motor" ,BLD_T0  , BLD_T1  ,CH_NULL,15,true);
MotorWrapper spareMotor(      "Spare Motor"      ,RPR0,     RPR1,    CH_NULL,15,true);
// Create Leds
LedWrapper leftLeds ("Left LEDs" ,LT1,CH_LED1,true);
LedWrapper rightLeds("Right LEDs",LT2,CH_LED2,true);
LedWrapper cabLeds  ("Cab LEDs"  ,LT3,CH_LED3,true);
//create Servos
ServoWrapper steeringServo  ("Steering Servo"  ,steeringServoPin,   90, 50, 130 ,true);
ServoWrapper attachmentServo("Attachment Servo",attachmentServoPin, 10, 10, 115 ,true);




int buttonSwitchTime = 0;

bool auxLedsOn = false;
using RobotControls::Controls;
using RobotControls::ReadBluepadControls;
Controls controller;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    
    if (i == BP32_MAX_GAMEPADS) BP32.enableNewBluetoothConnections(false);

    
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad() {

  if (!gMacro.isActive()) 
  {
    if (controller.rightBumper) gMacro.start(MacroType::StepRight);
    if (controller.leftBumper)  gMacro.start(MacroType::StepLeft);
    if (controller.a) gMacro.start(MacroType::RotateRightStep);
    if (controller.b)  gMacro.start(MacroType::RotateLeftStep);
  }


  if (!runMacros()) {
    // normal control path
    processThrottle(controller.axisY);
    processSteering(controller.axisX);
    processTool();
  }

  lastInputDelta = 0;

}

void processTool()
{

  float lift = controller.axisRY;
  float tilt = controller.axisRX;

  liftMotor.movePWM(lift,50);
  tiltMotor.movePWM(-tilt,50);


  if (controller.rightTrigger) attachmentMotor.forward(50);
  else if (controller.leftTrigger) attachmentMotor.reverse(50);


  if (controller.x.rising()) {
    attachmentServo.moveTo(attachmentServo.Position() < 60 ? 115 : 10);
  }


}

void processThrottle(float axisValue) {
  float adjustedThrottleValue = -axisValue;
  rearDriveMotor.movePWM(adjustedThrottleValue,50);
  frontDriveMotor.movePWM(adjustedThrottleValue,50);
}


static float applyExpo(float x, float expo)
{
  // x in [-1..1]
  // mix linear and cubic: y = (1-expo)*x + expo*x^3
  return (1.0f - expo) * x + expo * x * x * x;
}

void processSteering(float axisValue) {

    float x = axisValue * 2;
    if (x >  1.0f) x =  1.0f;
    if (x < -1.0f) x = -1.0f;

    // 2) Non-linear response
    float steeringValue = applyExpo(x, 0.45f);  // 0 = linear, 0.3-0.7 typical, higher = softer center

    steeringServo.moveTo(steeringValue);

  
}


struct LedState {
  uint8_t level;   // 0..255
  bool blink;      // turn signal
};


// Blink timing state
static uint32_t s_lastBlinkToggleMs = 0;
static bool s_blinkOn = true;

    // --- helpers ---
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}
static inline uint8_t lerpU8(float t01, uint8_t a, uint8_t b) {
  t01 = clampf(t01, 0.0f, 1.0f);
  return (uint8_t)(a + (b - a) * t01 + 0.5f);
}

static inline void applyLedState(LedWrapper& w, const LedState& s, bool blinkOnNow)
{
  // Gate brightness if blinking
  uint8_t out = s.blink ? (blinkOnNow ? s.level : 0) : s.level;

  // Use PWM if available (your setPWM degrades to on/off if PWM disabled)
  w.setPWM(out);
}

void updateLeds(float throttle, float steer)
{



  // ---- tuneables ----
  const float THROTTLE_DB = 0.08f;
  const float STEER_DB    = 0.25f;

  const uint32_t BLINK_PERIOD_MS = 350; // toggle interval

  const uint8_t RUN_CAB   = 50;  // idle/running brightness
  const uint8_t RUN_REAR  = 35;

  const uint8_t CAB_FWD_MIN = 90;   // when moving forward
  const uint8_t CAB_FWD_MAX = 255;

  const uint8_t REVERSE_BRIGHT = 255;
  const uint8_t TURN_BRIGHT = 255;

  // ---- clamp inputs ----
  throttle = clampf(throttle, -1.0f, 1.0f);
  steer    = clampf(steer,    -1.0f, 1.0f);


  const bool movingFwd = throttle >   THROTTLE_DB;
  const bool movingRev = throttle <  -THROTTLE_DB;

  const bool turningL = steer < -STEER_DB;
  const bool turningR = steer >  STEER_DB;


  // ---- base states: running lights ----
  LedState cab  { RUN_CAB,  false };
  LedState left { RUN_REAR, false };
  LedState right{ RUN_REAR, false };

  // ---- forward / reverse behavior ----
  if (movingFwd) {
    // scale cab brightness with forward throttle
    float t01 = (throttle - THROTTLE_DB) / (1.0f - THROTTLE_DB); // deadband..1 => 0..1
    cab.level = lerpU8(t01, CAB_FWD_MIN, CAB_FWD_MAX);

    if (turningL) {left.blink  = true; left.level  = TURN_BRIGHT;}
    if (turningR) {right.blink = true; right.level = TURN_BRIGHT;}
  }
  else if (movingRev) {
    // reverse lights on rear both
    left.level  = REVERSE_BRIGHT;
    right.level = REVERSE_BRIGHT;
    left.blink = true;
    right.blink = true;
  }


  // ---- blink phase update ----

  if (left.blink || right.blink) {
    if (LOOP_START_NOW - s_lastBlinkToggleMs >= BLINK_PERIOD_MS) {
      s_lastBlinkToggleMs = LOOP_START_NOW;
      s_blinkOn = !s_blinkOn;
    }
  } else {
    // Reset so the next time you start blinking it begins ON
    s_blinkOn = true;
    s_lastBlinkToggleMs = LOOP_START_NOW;
  }

  // ---- apply ----
  applyLedState(cabLeds,  cab,  s_blinkOn);
  applyLedState(leftLeds, left, s_blinkOn);
  applyLedState(rightLeds,right,s_blinkOn);

}


void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        lastInputTime = LOOP_START_NOW;
        controller.updateFrom(ReadBluepadControls(myController));
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}




static const float STEER_LEFT  = -1;   // your servo min
static const float STEER_RIGHT = 1;  // your servo max
static const float STEER_CENTER = 0;

static const uint32_t STEP_ARC_MS = 200;       // each arc duration
static const uint32_t STEP_SETTLE_MS = 300;       // each arc duration
static const uint32_t STEP_TURN_MS = 400;

bool runMacros()
{


  if (!gMacro.isActive()) return false;

  uint32_t now = LOOP_START_NOW;
  uint32_t dt = now - gMacro.t0;


  // helpers (lambdas) local to runMacros()
  auto nextPhase = [&]() {
    gMacro.phase++;
    gMacro.t0 = now;
  };

  // returns true when timer elapsed (and it advanced the phase)
  auto waitNextPhase = [&](uint32_t timerMs) -> bool {
    if (dt >= timerMs) { nextPhase(); return true; }
    return false;
  };

  auto drivePWM = [&](int pwm, uint32_t timerMs) {
    rearDriveMotor.movePWM(pwm, timerMs);
    frontDriveMotor.movePWM(pwm, timerMs);
  };

  auto forward = [&](int pwm, uint32_t timerMs) { drivePWM(+pwm, timerMs); };
  auto backward = [&](int pwm, uint32_t timerMs) { drivePWM(-pwm, timerMs); };

  auto left   = [&]() { steeringServo.moveTo(STEER_LEFT);   };
  auto right  = [&]() { steeringServo.moveTo(STEER_RIGHT);  };
  auto center = [&]() { steeringServo.moveTo(STEER_CENTER); };


  if (gMacro.active == MacroType::RotateRightStep) {
    switch (gMacro.phase) {
      case 0:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 1: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 2:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 3: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 4:
        center();
        gMacro.stop();
        break;
    }
  }

  if (gMacro.active == MacroType::RotateLeftStep) {
    switch (gMacro.phase) {
      case 0:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 1: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 2:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 3: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 4:
        center();
        gMacro.stop();
        break;
    }
  }


  if (gMacro.active == MacroType::StepRight) {
    switch (gMacro.phase) {
      case 0:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 1: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 2:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 3: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 4:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 5: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 6:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 7: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 8:
        center();
        waitNextPhase(STEP_SETTLE_MS);
        break;
      
      case 9:
        gMacro.stop();
        break;
    }
  }

  if (gMacro.active == MacroType::StepLeft) {
    switch (gMacro.phase) {
      case 0:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 1: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 2:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 3: 
        backward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 4:
        left();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 5: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 6:
        right();
        waitNextPhase(STEP_TURN_MS);
        break;

      case 7: 
        forward(200,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;

      case 8:
        center();
        waitNextPhase(STEP_SETTLE_MS);
        break;
      
      case 9:
        gMacro.stop();
        break;
    }
  }

  return true; // macro ran, so skip normal drive mapping this loop



}




const unsigned long INPUT_TIMEOUT = 250;  // ms — adjust if needed
const unsigned long MOTOR_TIMEOUT = 5000;  // ms — adjust if needed
const unsigned long LED_TIMEOUT = 10000;  // ms — adjust if needed
const unsigned long SERVO_TIMEOUT = 15000;  // ms — adjust if needed
bool motorsActive;
bool ledsActive;
bool servoActive;

// Arduino setup function. Runs in CPU 1
void setup() {

  Serial.begin(115200);
  LOOP_START_NOW = millis();

  // Motors
  rearDriveMotor.begin();
  frontDriveMotor.begin();
  tiltMotor.begin();
  liftMotor.begin();
  attachmentMotor.begin();
  spareMotor.begin();
  motorsActive = true;

  // Leds
  leftLeds.begin(false);
  rightLeds.begin(false);
  cabLeds.begin(false);
  ledsActive = true;

  //servos
  steeringServo.begin();
  attachmentServo.begin();
  servoActive=true;

  
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks and start scanning
  BP32.setup(&onConnectedController, &onDisconnectedController);
  

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);

  lastInputTime = LOOP_START_NOW;  // initialize failsafe timer
}





// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  LOOP_START_NOW = millis();
  lastInputDelta = LOOP_START_NOW - lastInputTime;

  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    //Serial.printf("dataUpdated Start\n");
    processControllers();
    
    motorsActive = true;
    ledsActive = true;
    servoActive = true;
    //Serial.printf("dataUpdated End\n");
  }
  else { 
      // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1); 
  }

  processGamepad();

  

  
  //Serial.printf("UPDATES\n");

  if (motorsActive)  
  {
      // Failsafe check: if no input for too long, stop motors
    if (lastInputDelta > INPUT_TIMEOUT + MOTOR_TIMEOUT) 
    {
      rearDriveMotor.off();
      frontDriveMotor.off();
      liftMotor.off();
      tiltMotor.off();
      spareMotor.off();
      attachmentMotor.off();
      motorsActive=false;
    }
    else
    {
      rearDriveMotor.update();
      frontDriveMotor.update();
      liftMotor.update();
      tiltMotor.update();
      spareMotor.update();
      attachmentMotor.update();
    }
  }

    
  if (servoActive)  
  {
      // Failsafe check: if no input for too long, stop servos
    if (lastInputDelta > INPUT_TIMEOUT + SERVO_TIMEOUT) 
    {
      attachmentServo.detach();
      steeringServo.detach();
      servoActive=false;
    }
    else
    {
      //servo updates
      attachmentServo.update();
      steeringServo.update();
    }
  }

  if (ledsActive)  
  {
      // Failsafe check: if no input for too long, stop servos
    if (lastInputDelta > INPUT_TIMEOUT + LED_TIMEOUT) 
    {
      cabLeds.off();
      leftLeds.off();
      rightLeds.off();
      ledsActive=false;
    }
    else
    {
      //led updates

      float throttle = rearDriveMotor.normalizedSpeed();
      float steering = steeringServo.normalizedPosition();
      updateLeds(throttle,steering); 
    }
  }



  
}

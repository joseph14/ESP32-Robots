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

#define TXPin 1
#define RXPin 3

#define GIO23Pin 23
#define GIO22Pin 22
#define GIO21Pin 21
#define GIO19Pin 19

#define LT_MTR0 4   
#define LT_MTR1 2    
#define RT_MTR0 12 
#define RT_MTR1 13  

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
static const int CH_PUMP = 9;
static const int CH_MTR = 10;

static const int CH_LED1 = 12;
static const int CH_LED2 = 13;
static const int CH_LED3 = 14;

static const int CH_NULL = -1;


// Create motors
MotorWrapper rearLeftMotor(  "Rear Drive Motor" ,LT_MTR0 , LT_MTR1 ,CH_MTR ,15,true);
MotorWrapper rearRightMotor( "Front Drive Motor",RT_MTR0 , RT_MTR1 ,CH_MTR ,15,true);
MotorWrapper frontLeftMotor(       "Lift Motor"       ,LBL_MTR0, LBL_MTR1,CH_MTR ,15,true);
MotorWrapper frontRightMotor(       "Tilt Motor"       ,RBL_MTR1, RBL_MTR0,CH_MTR ,15,true);
MotorWrapper vacuumPumpMotor( "Attachment Motor" ,BLD_T0  , BLD_T1  ,CH_PUMP,15,true);
MotorWrapper spareMotor(      "Spare Motor"      ,RPR0,     RPR1,    CH_NULL,15,true);
// Create Leds
LedWrapper leftLeds ("Left LEDs" ,LT1,CH_LED1,true);
LedWrapper rightLeds("Right LEDs",LT2,CH_LED2,true);
LedWrapper cabLeds  ("Cab LEDs"  ,LT3,CH_LED3,true);
//create Servos
ServoWrapper rearLeftServo ("Spare Servo",GIO23Pin,  90, 11.62, 168.38 ,true);
ServoWrapper rearRightServo ("Spare Servo",GIO22Pin,  90, 11.62, 168.38 ,true);
ServoWrapper frontLeftServo ("Spare Servo",GIO21Pin,  90, 11.62, 168.38 ,true);
ServoWrapper frontRightServo ("Spare Servo",GIO19Pin,  90, 11.62, 168.38 ,true);

 
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

int controlMode = 0;

float vaccuumLevel = 0;


static constexpr float AXIS_MAX = 512.0f;   // set to your real max magnitude (e.g. 2047, 32767, etc.)
void processGamepad() {

  if (!gMacro.isActive()) 
  {
    if (controller.rightBumper) gMacro.start(MacroType::StepRight);
    if (controller.leftBumper)  gMacro.start(MacroType::StepLeft);
    if (controller.x) gMacro.start(MacroType::RotateLeftStep);
    if (controller.y)  gMacro.start(MacroType::RotateRightStep);

  }


  if (!runMacros()) {
    // normal control path
    if (controller.a.rising() && controller.rightTrigger) controlMode++;
    if (controller.b.rising() && controller.rightTrigger) controlMode--;
    if (controlMode > 9) 
    {
        controlMode=0;
    }

    if (controlMode < 0) 
    {
        controlMode=0;
    }

    float x = controller.axisX ;
    float y = controller.axisY ;
    processThrottle(x,y);
    processSteering(x,y);


    if (controller.dpadUp.rising()) vaccuumLevel+=0.1f;
    if (controller.dpadDown.rising()) vaccuumLevel-=0.1f;
    vaccuumLevel = clampf(vaccuumLevel, 0.0f, 1.0f);
    vacuumPumpMotor.movePWM(vaccuumLevel,50);
  }

  lastInputDelta = 0;

}

// axisValue is assumed centered at 0 (negative = left, positive = right)
// TUNE THESE:

static constexpr float EXPO     = 0.45f;     // 0 = linear, 0.3-0.7 typical, higher = softer center

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

static const float THROTTLE_SCALE[] = {
  0.55f,
  0.60f,
  0.65f,
  0.70f,
  0.75f,
  0.80f,
  0.85f,
  0.90f,
  0.95f,
  1.00f
};


static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float lerpf(float a, float b, float t) {
  t = clampf(t, 0.0f, 1.0f);
  return a + (b - a) * t;
}

void processThrottle(float x, float y) {

  float magnitude = hypotf(x, y);

  magnitude *= clampf(fabs(y)*5,0,1);

  int mode = controlMode;
  if (mode < 0) mode = 0;
  if (mode > 9) mode = 9;

  float adjustedThrottleValue = -magnitude * THROTTLE_SCALE[mode]*sign(y);

  rearLeftMotor.movePWM(adjustedThrottleValue, 50);
  rearRightMotor.movePWM(adjustedThrottleValue, 50);
  frontLeftMotor.movePWM(adjustedThrottleValue, 50);
  frontRightMotor.movePWM(adjustedThrottleValue, 50);

  char line[120];
  snprintf(line, sizeof(line),
      "mag=%7.3f throttle=%7.3f y=%7.3f mode=%d",
      magnitude, adjustedThrottleValue, y, mode);

  Serial.println(line);
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


static constexpr float L=1.0f;
static constexpr float W=1.0f;


  float wrap180(float deg) {
    while (deg > 90.0f) deg -= 180.0f;
    while (deg < -90.0f) deg += 180.0f;
    return deg;
  }

  // Convert raw wheel direction into servo-relative angle
  float applyMountOffset(float rawDeg, float mountDeg) {
    return wrap180(rawDeg - mountDeg);
  }



  // Tangent direction for wheel at (x,y), center of rotation at (0,R)
  float rawWheelAngleDeg(float x, float y, float R) {
    float signOfR = sign(R);
    return atan2f(R - y,x*signOfR) * 180.0f / M_PI * signOfR;
  }

  

void turnWheelsToRadius(float R){


    float flRaw = rawWheelAngleDeg(+L, +W, R);
    float frRaw = rawWheelAngleDeg(+L, -W, R);
    float blRaw = rawWheelAngleDeg(-L, +W, R);
    float brRaw = rawWheelAngleDeg(-L, -W, R);

    float fl = applyMountOffset(flRaw, 45)/90;
    float fr = applyMountOffset(frRaw, -45)/90;
    float bl = applyMountOffset(blRaw, -45)/90;
    float br = applyMountOffset(brRaw, 45)/90;

      rearLeftServo.moveTo(bl);
      rearRightServo.moveTo(br);
      frontLeftServo.moveTo(fl);
      frontRightServo.moveTo(fr);

  char line[120];

  snprintf(line, sizeof(line),
          "R=%7.3f   FL=%7.2f  FR=%7.2f  BL=%7.2f  BR=%7.2f FL=%7.2f  FR=%7.2f  BL=%7.2f  BR=%7.2f",
           R, fl, fr, bl, br, flRaw, frRaw,blRaw,brRaw);

  Serial.println(line);
  
}

void processSteering(float x,float y){

    float a = x;   // normalized steering input in [-1..1]
    float R = controllerToRadius(a);

    turnWheelsToRadius(R);
}


struct LedState {
  uint8_t level;   // 0..255
  bool blink;      // turn signal
};


// Blink timing state
static uint32_t s_lastBlinkToggleMs = 0;
static bool s_blinkOn = true;

    // --- helpers ---

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




static const float STEER_LEFT  = -0.01;   // your servo min
static const float STEER_RIGHT = 0.01;  // your servo max
static const float STEER_CENTER = 100000;

static const uint32_t STEP_ARC_MS = 100;       // each arc duration
static const uint32_t STEP_SETTLE_MS = 200;       // each arc duration
static const uint32_t STEP_TURN_MS = 250;

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

  auto drivePWM = [&](float pwm, uint32_t timerMs) {
    rearLeftMotor.movePWM(pwm, timerMs);
    rearRightMotor.movePWM(pwm, timerMs);
    frontRightMotor.movePWM(pwm, timerMs);
    frontRightMotor.movePWM(pwm, timerMs);
  };

  auto forward = [&](float pwm, uint32_t timerMs) { drivePWM(+pwm, timerMs); };
  auto backward = [&](float pwm, uint32_t timerMs) { drivePWM(-pwm, timerMs); };

  auto left   = [&]() { turnWheelsToRadius(STEER_LEFT);   };
  auto right  = [&]() { turnWheelsToRadius(STEER_RIGHT);  };
  auto center = [&]() { turnWheelsToRadius(STEER_CENTER); };


  if (gMacro.active == MacroType::RotateRightStep) {
    switch (gMacro.phase) {
      case 0:
        right();
        forward(0.9,STEP_TURN_MS);
        waitNextPhase(STEP_TURN_MS);
        break;
      case 1:
        gMacro.stop();
        break;
    }
  }

  if (gMacro.active == MacroType::RotateLeftStep) {
    switch (gMacro.phase) {
      case 0:
        left();
        forward(0.9,STEP_TURN_MS);
        waitNextPhase(STEP_TURN_MS);
        break;
      case 1:
        gMacro.stop();
        break;
    }
  }


  if (gMacro.active == MacroType::StepRight) {
    switch (gMacro.phase) {
      case 0:
        right();
        forward(0.9,STEP_TURN_MS);
        waitNextPhase(STEP_TURN_MS);
        break;
      case 1:
        center();
        forward(0.9,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;
      case 2:
        gMacro.stop();
        break;
    }
  }

  if (gMacro.active == MacroType::StepLeft) {
    switch (gMacro.phase) {
      case 0:
        left();
        forward(0.9,STEP_TURN_MS);
        waitNextPhase(STEP_TURN_MS);
        break;
      case 1:
        center();
        forward(0.9,STEP_ARC_MS);
        waitNextPhase(STEP_ARC_MS);
        break;
      case 2:
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
  rearLeftMotor.begin();
  rearRightMotor.begin();
  frontRightMotor.begin();
  frontLeftMotor.begin();
  vacuumPumpMotor.begin();
  spareMotor.begin();
  motorsActive = true;

  // Leds
  leftLeds.begin(false);
  rightLeds.begin(false);
  cabLeds.begin(false);
  ledsActive = true;

  //servos
  rearLeftServo.begin();
  rearRightServo.begin();
  frontLeftServo.begin();
  frontRightServo.begin();

  servoActive=true;

  Serial.printf("atan2f(1,0) %f\n", atan2f(1.0f,0.0f));

  
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
      rearLeftMotor.off();
      rearRightMotor.off();
      frontLeftMotor.off();
      frontRightMotor.off();
      spareMotor.off();
      vacuumPumpMotor.off();
      motorsActive=false;
    }
    else
    {
      rearLeftMotor.update();
      rearRightMotor.update();
      frontLeftMotor.update();
      frontRightMotor.update();
      spareMotor.update();
      vacuumPumpMotor.update();
    }
  }

    
  if (servoActive)  
  {
      // Failsafe check: if no input for too long, stop servos
    if (lastInputDelta > INPUT_TIMEOUT + SERVO_TIMEOUT) 
    {
      rearLeftServo.detach();
      rearRightServo.detach();
      frontLeftServo.detach();
      frontRightServo.detach();

      servoActive=false;
    }
    else
    {
      //servo updates
      rearLeftServo.update();
      rearRightServo.update();
      frontLeftServo.update();
      frontRightServo.update();

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

      float throttle = rearLeftMotor.normalizedSpeed();
      float steering = rearLeftServo.normalizedPosition();
      updateLeds(throttle,steering); 
    }
  }



  
}

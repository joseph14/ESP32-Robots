#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>
#include <type_traits>
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


struct LedState;  // forward declaration

class GenericWrapper {
public:
  GenericWrapper(const char* name, bool debug = false)
  : _name(name), _debug(debug) {}

  void setDebug(bool enable) { _debug = enable; }
  bool debugEnabled() const { return _debug; }

  const char* name() const { return _name; }

protected:
  void debugMsg(const char* msg) const {
    if (!_debug) return;
    Serial.printf("[%lu][%s] %s\n", LOOP_START_NOW, _name, msg);
  }

  template<typename... Args>
  void debugf(const char* fmt, Args... args) const {
    if (!_debug) return;
    Serial.printf("[%lu][%s] ", LOOP_START_NOW, _name);
    Serial.printf(fmt, args...);
    Serial.println();
  }

private:
  const char* _name;
  bool _debug;
};


class MotorWrapper : public GenericWrapper {
public:

  static inline bool validCh(int ch) { return ch >= 0; }

  static const int PWM_FREQ = 20000;  // 20 kHz (quiet)
  static const int PWM_RES  = 8;      // 0..255

  enum class State : uint8_t {
    Off = 0,
    Forward,
    Reverse,
    PWMForward,
    PWMReverse
  };

  MotorWrapper(const char* name, int pin0, int pin1, int ch, int deadband = 15, bool debug = false)
  : GenericWrapper(name, debug), _p0(pin0), _p1(pin1), _ch(ch), _deadband(deadband) {}

  void begin() {
    pinMode(_p0, OUTPUT);
    pinMode(_p1, OUTPUT);
    _pwmEnabled = validCh(_ch);

    if (_pwmEnabled) {
      // Configure + attach channels
      ledcSetup(_ch, PWM_FREQ, PWM_RES);
    } 

    off();
    _state = State::Off;
    _lastDuty = 0;

    debugf("Begin (p0=%d, p1=%d, ch=%d, pwm=%s, deadband=%d)",
           _p0, _p1, _ch, _pwmEnabled ? "yes" : "no", _deadband);
  }

  bool pwmEnabled() const { return _pwmEnabled; }

  // Query
  State state() const { return _state; }
  bool isOff() const { return _state == State::Off; }
  bool isOn() const { return _state != State::Off; }
  bool isForward() const { return _state == State::Forward ||  _state == State::PWMForward; }
  bool isReverse() const { return _state == State::Reverse || _state == State::PWMReverse; }
  int lastPwm() const { return _lastDuty; }   // 0..255 (0 if digital or off)

 
  // PWM move: velocity in [-1..1]
  void movePWM(float norm_velocity, unsigned long holdMs) {
      movePWM((int)(norm_velocity*255),holdMs);

  }



  // PWM move: velocity in [-255..255]
  void movePWM(int velocity, unsigned long holdMs) {
    
    _timer = LOOP_START_NOW;
    _holdTime = holdMs;

    // If PWM isn't configured, degrade gracefully to digital
    if (!_pwmEnabled) {
      if (velocity > _deadband) forward(holdMs);
      else if (velocity < -_deadband) reverse(holdMs);
      else off();
      return;
    }

    if (velocity > _deadband) {
      int duty = clamp255(velocity);
      if (_lastDuty != duty || _state != State::PWMForward) {
        ensureDetatch1();
        ensureAttach0();
        ledcWrite(_ch, duty);

        _lastDuty = duty;
        _speed = duty;
        _state = State::PWMForward;
        debugf("Move +%d (PWM=%d)", velocity, duty);
      }
    } else if (velocity < -_deadband) {
      int duty = clamp255(-velocity);
      if (_lastDuty != duty || _state != State::PWMReverse) {
        ensureDetatch0();
        ensureAttach1();
        ledcWrite(_ch, duty);

        _lastDuty = duty;
        _speed = -duty;
        _state = State::PWMReverse;
        debugf("Move %d (PWM=%d)", velocity, duty);
      }
    } else {
      if (_state != State::Off) {
        ensureDetatch1();
        ensureDetatch0();
        
        _lastDuty = 0;
        _speed = 0;
        _state = State::Off;
        debugf("Move %d -> Off (deadband)", velocity);
      }
    }
  }

  


  void ensureDetatch0(){
      if (_p0Attached) {
        ledcDetachPin(_p0);
        _p0Attached=false;
      }
      digitalWrite(_p0, LOW);
  }
  void ensureDetatch1(){
      if (_p1Attached) {
        ledcDetachPin(_p1);
        _p1Attached=false;
      }
      digitalWrite(_p1, LOW);
  }

  void ensureAttach0(){
      if (!_p0Attached){
        ledcAttachPin(_p0, _ch);
        _p0Attached=true;
      } 
  }
  void ensureAttach1(){
      if (!_p1Attached){
        ledcAttachPin(_p1, _ch);
        _p1Attached=true;
      } 
  }


  // Digital helpers (full speed)
  void forward(unsigned long holdMs) {

    if (_pwmEnabled) {
      // full speed forward using PWM hardware
      movePWM(255,holdMs);
      return;
    }

    _timer = LOOP_START_NOW;
    _holdTime = holdMs;

    if (_state == State::Forward) return;
    _speed = 255;

    digitalWrite(_p0, HIGH);
    digitalWrite(_p1, LOW);
    
    _lastDuty = 0;
    _state = State::Forward;
    debugMsg("Forward");
  }

  void reverse(unsigned long holdMs) {
    
    if (_pwmEnabled) {
      // full speed reverse using PWM hardware
      movePWM(-255,holdMs);
      return;
    }

    _timer = LOOP_START_NOW;
    _holdTime = holdMs;

    if (_state == State::Reverse) return;
    _speed = -255;

    digitalWrite(_p0, LOW);
    digitalWrite(_p1, HIGH);
    

    _lastDuty = 0;
    _state = State::Reverse;
    debugMsg("Reverse");
  }

  void off() {

    if (_pwmEnabled) {
      movePWM(0,0);
      return;
    }

    if (_state == State::Off) return;

    _speed = 0;

    digitalWrite(_p0, LOW);
    digitalWrite(_p1, LOW);

    _lastDuty = 0;
    _state = State::Off;
    debugMsg("Off");
  }
  void update(){
    if (LOOP_START_NOW - _timer >= _holdTime) {
      off();
    }
  }


  void stop() { off(); }

  float normalizedSpeed() { return (float) _speed / 255.0;  }


private:
  int _p0, _p1;
  bool _p0Attached, _p1Attached;
  int _ch;
  bool _pwmEnabled = false;

  int _deadband;
  State _state = State::Off;
  int _lastDuty = 0;
  int _speed = 0;
  unsigned long _timer = 0;
  unsigned long _holdTime = 0;

  static int clamp255(int duty) {
    if (duty > 255) return 255;
    if (duty < 0)   return 0;
    return duty;
  }
};




class LedWrapper : public GenericWrapper {
public:
  enum class State : uint8_t { Off = 0, On, PWM };

  // ch = LEDC channel (0..15). Use -1 for no PWM (digital only).
  LedWrapper(const char* name, int pin, int ch = -1, bool debug = false)
  : GenericWrapper(name, debug), _pin(pin), _ch(ch) {}

  // freq/res apply only if PWM enabled
  void begin(bool startOn = false, int freq = 1000, int resBits = 8) {
    pinMode(_pin, OUTPUT);

    _pwmEnabled = (_ch >= 0);

    if (_pwmEnabled) {
      _freq = freq;
      _resBits = resBits;
      _maxDuty = (1 << _resBits) - 1;

      ledcSetup(_ch, _freq, _resBits);
      ledcAttachPin(_pin, _ch);

      // start state
      if (startOn) setPWM(_maxDuty);
      else         setPWM(0);

      debugf("Begin (pin=%d, ch=%d, pwm=yes, freq=%d, res=%d, startOn=%s)",
             _pin, _ch, _freq, _resBits, startOn ? "true" : "false");
    } else {
      startOn ? on() : off();
      debugf("Begin (pin=%d, pwm=no, startOn=%s)", _pin, startOn ? "true" : "false");
    }
  }

  // Digital semantics (still works in PWM mode)
  void on() {

    if (_state != State::On){

      if (_pwmEnabled) {
        setPWM(_maxDuty);
        return;
      }
      if (_state != State::On) {
        digitalWrite(_pin, HIGH);
        _state = State::On;
        _duty = 255;
        debugMsg("On");
      }
    }

  }

  void off() {

    if (_state != State::Off){

      if (_pwmEnabled) {
        setPWM(0);
        return;
      }
      if (_state != State::Off) {
        digitalWrite(_pin, LOW);
        _state = State::Off;
        _duty = 0;
        debugMsg("Off");
      }
    }
  }

  void set(bool value) { value ? on() : off(); }

  // PWM brightness control: duty 0..255 (mapped to resBits)
  void setPWM(uint8_t duty255) {
    if (!_pwmEnabled) {
      // degrade: treat any nonzero as ON
      duty255 ? on() : off();
      return;
    }

    if (_duty == duty255 && _state == State::PWM) return;


    _duty = duty255;

    // Map 0..255 -> 0..maxDuty
    uint32_t duty = (uint32_t)_duty * (uint32_t)_maxDuty / 255u;

    if (duty == 0 && _state == State::Off) return;
    ledcWrite(_ch, duty);

    _state = (duty == 0) ? State::Off : State::PWM;
    debugf("PWM duty=%u (scaled=%u/%u)", _duty, (unsigned)duty, (unsigned)_maxDuty);
  }

  // Optional: “hardware blink” without loop timing.
  // hz can be fractional, but keep it sensible (e.g. 0.5 to 10).
  void blinkHardware(float hz, uint8_t duty255 = 128, int resBits = 8) {
    // Requires PWM enabled
    if (!_pwmEnabled) return;

    // If hz <= 0, stop blinking and hold current duty
    if (hz <= 0.0f) {
      setPWM(duty255);
      return;
    }

    // Use LEDC frequency as blink frequency (square wave if duty is 50%)
    // NOTE: In this mode, duty sets ON fraction of each blink cycle.
    _resBits = resBits;
    _maxDuty = (1 << _resBits) - 1;
    _freq = (int)(hz + 0.5f); // round to int Hz; keep simple

    ledcSetup(_ch, _freq, _resBits);

    // duty255 is still brightness within "on" portion only if you're using an external transistor;
    // for a simple LED on GPIO, duty here is effectively "duty-cycle of blink".
    uint32_t duty = (uint32_t)duty255 * (uint32_t)_maxDuty / 255u;
    ledcWrite(_ch, duty);

    _state = (duty == 0) ? State::Off : State::PWM;
    _duty = duty255;
    debugf("Hardware blink freq=%dHz duty=%u", _freq, _duty);
  }

  // Query
  bool pwmEnabled() const { return _pwmEnabled; }
  bool isOn() const { return _state != State::Off; }
  State state() const { return _state; }
  int pin() const { return _pin; }
  int channel() const { return _ch; }
  uint8_t duty() const { return _duty; } // 0..255

private:
  int _pin;
  int _ch;                // -1 means no PWM
  bool _pwmEnabled = false;

  int _freq = 1000;
  int _resBits = 8;
  uint32_t _maxDuty = 255;

  State _state = State::Off;
  uint8_t _duty = 0;      // store 0..255 for convenience
};

class ServoWrapper : public GenericWrapper {
public:
  enum class State : uint8_t {
    Detached = 0,
    Holding
  };

  ServoWrapper(const char* name, int pin, int defaultPos, int minPos, int maxPos, bool debug = false)
  : GenericWrapper(name, debug),   _pin(pin),   _defaultPos(defaultPos),    _minPos(minPos),    _maxPos(maxPos)
    {
        if (_minPos > _maxPos) { 
          int t=_minPos; 
          _minPos=_maxPos; 
          _maxPos=t; 
        }
      _defaultPos = clampInt(_defaultPos, _minPos, _maxPos);
      _midPos = 0.5f * (float)(_maxPos + _minPos);
      _halfRange = 0.5f * (float)(_maxPos - _minPos);
    }

  void begin() {
    _position = _defaultPos;
    _servo.attach(_pin);
    _servo.write(_position);
    _servo.detach();
    _state = State::Detached;

    debugf("Initialized at position %d", _position);
  }


  void moveTo(float x) {

    int angle = (int)(x * _halfRange+_midPos);
    moveTo(angle); //angles are clamped in this function

  }


  void moveTo(int setPosition) {
    setPosition =  clampInt(setPosition, _minPos, _maxPos);
    if (_position != setPosition){
      _position = setPosition;
      ensureAttached();
      _servo.write(_position);
      _state = State::Holding;
      _timedDetach = false;
      debugf("MoveTo: %d", _position);
    }


  }

  void moveFor(int setPosition, unsigned long holdMs) {
    setPosition =  clampInt(setPosition, _minPos, _maxPos);
    _timer = LOOP_START_NOW;
    _holdTime = holdMs;
    _timedDetach = true;

    if (_position != setPosition){
      _position = setPosition;
      ensureAttached();
      _servo.write(_position);
  

      _state = State::Holding;

      
      debugf("MoveFor: %d hold=%lu", _position, holdMs);
    }
  }

  void update() {
    if (_timedDetach && _state == State::Holding) {
      if (LOOP_START_NOW - _timer >= _holdTime) {
        detach();
      }
    }
  }

  void detach() {
    _servo.detach();
    _state = State::Detached;
    _timedDetach = false;

    debugMsg("Detached");
  }

  int Position() { return _position; }
  float normalizedPosition() { return ((float) _position - _midPos) / _halfRange; }  

private:
  Servo _servo;
  int _pin;
  int _defaultPos;
  int _maxPos;
  int _minPos;
  float _midPos;
  float _halfRange;

  State _state = State::Detached;

  int _position;

  unsigned long _timer = 0;
  unsigned long _holdTime = 0;
  bool _timedDetach = false;

  void ensureAttached() {
    if (_state == State::Detached) {
      _servo.attach(_pin);
      debugMsg("Attached");
    }
  }



  static int clampInt(int p, int lo, int hi)
  {
    if (p < lo) return lo;
    if (p > hi) return hi;
    return p;
  }
};

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

template <typename T>
class Tracked {
    static_assert(
    std::is_arithmetic<T>::value || std::is_enum<T>::value,
    "Tracked<T> requires arithmetic or enum type"
  );
public:
  Tracked() : _value(), _old(), _changed(false) {}
  explicit Tracked(const T& initial) : _value(initial), _old(initial), _changed(false) {}

  bool set(const T& v1) {
    if (v1 == _value) { _changed = false; return false; }
    _old = _value;
    _value = v1;
    _changed = true;
    return true;
  }

  Tracked& operator=(const T& v1) { set(v1); return *this; }

  const T& value() const { return _value; }
  const T& old()   const { return _old; }

  bool changed() const { return _changed; }
  void clearChanged() { _changed = false; }

  // Generic “direction” edges: works for bool, ints, floats, enums (if comparable)
  bool rising()  const { return _changed && (_value > _old); }
  bool falling() const { return _changed && (_value < _old); }

  // Useful extras
  bool wentTo(const T& v1)   const { return _changed && (_value == v1); }
  bool cameFrom(const T& v1) const { return _changed && (_old == v1); }

  // Threshold crossing helpers (great for axes / deadbands)
  bool crossedUp(const T& threshold) const {
    return _changed && (_old < threshold) && (_value >= threshold);
  }
  bool crossedDown(const T& threshold) const {
    return _changed && (_old > threshold) && (_value <= threshold);
  }

  operator T() const { return _value; }

private:
  T _value;
  T _old;
  bool _changed;
};





float NormalizeAxis(int Axis){
  float norm = (float)Axis / 512.0f;
  if (norm>1) norm=1;
  if (norm<-1) norm =-1;
  return norm;
}

struct Controls {
  Tracked<float>  axisY, axisX, axisRY, axisRX;
  Tracked<bool> thumbL, thumbR;
  Tracked<bool> rightBumper, leftBumper;
  Tracked<bool> rightTrigger, leftTrigger;
  Tracked<bool> a, b, x, y;
  Tracked<int>  dpad;
  Tracked<bool> dpadUp, dpadDown;

  bool updateFrom(ControllerPtr ctl) {
    bool changed = false;

    changed |= axisY.set(NormalizeAxis(ctl->axisY()));
    changed |= axisX.set(NormalizeAxis(ctl->axisX()));
    changed |= axisRY.set(NormalizeAxis(ctl->axisRY()));
    changed |= axisRX.set(NormalizeAxis(ctl->axisRX()));

    changed |= thumbL.set(ctl->thumbL());
    changed |= thumbR.set(ctl->thumbR());

    changed |= rightBumper.set(ctl->r1());
    changed |= leftBumper.set(ctl->l1());

    changed |= rightTrigger.set(ctl->r2());
    changed |= leftTrigger.set(ctl->l2());

    changed |= a.set(ctl->a());
    changed |= b.set(ctl->b());
    changed |= x.set(ctl->x());
    changed |= y.set(ctl->y());

    int newDpad = ctl->dpad();
    changed |= dpad.set(newDpad);
    changed |= dpadUp.set(newDpad == 1);
    changed |= dpadDown.set(newDpad == 2);

    if (changed) {
      Serial.printf(
        "axisY=%.3f axisX=%.3f axisRY=%.3f axisRX=%.3f | "
        "thumbL=%d thumbR=%d | "
        "LB=%d RB=%d LT=%d RT=%d | "
        "A=%d B=%d X=%d Y=%d | "
        "dpad=%d (U=%d D=%d) | "
        "changed=%d\n",
        axisY.value(), axisX.value(), axisRY.value(), axisRX.value(),
        thumbL.value(), thumbR.value(),
        leftBumper.value(), rightBumper.value(), leftTrigger.value(), rightTrigger.value(),
        a.value(), b.value(), x.value(), y.value(),
        dpad.value(), dpadUp.value(), dpadDown.value(),
        changed
      );
    }

    return changed;
  }
};





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
        controller.updateFrom(myController);
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

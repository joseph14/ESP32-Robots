#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

extern unsigned long LOOP_START_NOW;

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

  template <typename... Args>
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
  static const int PWM_RES = 8;       // 0..255

  enum class State : uint8_t { Off = 0, Forward, Reverse, PWMForward, PWMReverse };

  MotorWrapper(const char* name, int pin0, int pin1, int ch, int deadband = 15, bool debug = false)
      : GenericWrapper(name, debug), _p0(pin0), _p1(pin1), _ch(ch), _deadband(deadband) {}

  void begin() {
    pinMode(_p0, OUTPUT);
    pinMode(_p1, OUTPUT);
    _pwmEnabled = validCh(_ch);

    if (_pwmEnabled) {
      ledcSetup(_ch, PWM_FREQ, PWM_RES);
    }

    off();
    _state = State::Off;
    _lastDuty = 0;

    debugf("Begin (p0=%d, p1=%d, ch=%d, pwm=%s, deadband=%d)", _p0, _p1, _ch,
           _pwmEnabled ? "yes" : "no", _deadband);
  }

  bool pwmEnabled() const { return _pwmEnabled; }

  State state() const { return _state; }
  bool isOff() const { return _state == State::Off; }
  bool isOn() const { return _state != State::Off; }
  bool isForward() const { return _state == State::Forward || _state == State::PWMForward; }
  bool isReverse() const { return _state == State::Reverse || _state == State::PWMReverse; }
  int lastPwm() const { return _lastDuty; }

  void movePWM(float norm_velocity, unsigned long holdMs) { movePWM((int)(norm_velocity * 255), holdMs); }

  void movePWM(int velocity, unsigned long holdMs) {
    _timer = LOOP_START_NOW;
    _holdTime = holdMs;

    if (!_pwmEnabled) {
      if (velocity > _deadband)
        forward(holdMs);
      else if (velocity < -_deadband)
        reverse(holdMs);
      else
        off();
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

  void ensureDetatch0() {
    if (_p0Attached) {
      ledcDetachPin(_p0);
      _p0Attached = false;
    }
    digitalWrite(_p0, LOW);
  }
  void ensureDetatch1() {
    if (_p1Attached) {
      ledcDetachPin(_p1);
      _p1Attached = false;
    }
    digitalWrite(_p1, LOW);
  }

  void ensureAttach0() {
    if (!_p0Attached) {
      ledcAttachPin(_p0, _ch);
      _p0Attached = true;
    }
  }
  void ensureAttach1() {
    if (!_p1Attached) {
      ledcAttachPin(_p1, _ch);
      _p1Attached = true;
    }
  }

  void forward(unsigned long holdMs) {
    if (_pwmEnabled) {
      movePWM(255, holdMs);
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
      movePWM(-255, holdMs);
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
      movePWM(0, 0);
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
  void update() {
    if (LOOP_START_NOW - _timer >= _holdTime) {
      off();
    }
  }

  void stop() { off(); }

  float normalizedSpeed() { return (float)_speed / 255.0; }

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
    if (duty < 0) return 0;
    return duty;
  }
};

class LedWrapper : public GenericWrapper {
public:
  enum class State : uint8_t { Off = 0, On, PWM };

  LedWrapper(const char* name, int pin, int ch = -1, bool debug = false)
      : GenericWrapper(name, debug), _pin(pin), _ch(ch) {}

  void begin(bool startOn = false, int freq = 1000, int resBits = 8) {
    pinMode(_pin, OUTPUT);

    _pwmEnabled = (_ch >= 0);

    if (_pwmEnabled) {
      _freq = freq;
      _resBits = resBits;
      _maxDuty = (1 << _resBits) - 1;

      ledcSetup(_ch, _freq, _resBits);
      ledcAttachPin(_pin, _ch);

      if (startOn)
        setPWM(_maxDuty);
      else
        setPWM(0);

      debugf("Begin (pin=%d, ch=%d, pwm=yes, freq=%d, res=%d, startOn=%s)", _pin, _ch, _freq,
             _resBits, startOn ? "true" : "false");
    } else {
      startOn ? on() : off();
      debugf("Begin (pin=%d, pwm=no, startOn=%s)", _pin, startOn ? "true" : "false");
    }
  }

  void on() {
    if (_state != State::On) {
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
    if (_state != State::Off) {
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

  void setPWM(uint8_t duty255) {
    if (!_pwmEnabled) {
      duty255 ? on() : off();
      return;
    }

    if (_duty == duty255 && _state == State::PWM) return;

    _duty = duty255;

    uint32_t duty = (uint32_t)_duty * (uint32_t)_maxDuty / 255u;

    if (duty == 0 && _state == State::Off) return;
    ledcWrite(_ch, duty);

    _state = (duty == 0) ? State::Off : State::PWM;
    debugf("PWM duty=%u (scaled=%u/%u)", _duty, (unsigned)duty, (unsigned)_maxDuty);
  }

  void blinkHardware(float hz, uint8_t duty255 = 128, int resBits = 8) {
    if (!_pwmEnabled) return;

    if (hz <= 0.0f) {
      setPWM(duty255);
      return;
    }

    _resBits = resBits;
    _maxDuty = (1 << _resBits) - 1;
    _freq = (int)(hz + 0.5f);

    ledcSetup(_ch, _freq, _resBits);

    uint32_t duty = (uint32_t)duty255 * (uint32_t)_maxDuty / 255u;
    ledcWrite(_ch, duty);

    _state = (duty == 0) ? State::Off : State::PWM;
    _duty = duty255;
    debugf("Hardware blink freq=%dHz duty=%u", _freq, _duty);
  }

  bool pwmEnabled() const { return _pwmEnabled; }
  bool isOn() const { return _state != State::Off; }
  State state() const { return _state; }
  int pin() const { return _pin; }
  int channel() const { return _ch; }
  uint8_t duty() const { return _duty; }

private:
  int _pin;
  int _ch;
  bool _pwmEnabled = false;

  int _freq = 1000;
  int _resBits = 8;
  uint32_t _maxDuty = 255;

  State _state = State::Off;
  uint8_t _duty = 0;
};

class ServoWrapper : public GenericWrapper {
public:
  enum class State : uint8_t { Detached = 0, Holding };

  ServoWrapper(const char* name, int pin, int defaultPos, int minPos, int maxPos, bool debug = false)
      : GenericWrapper(name, debug),
        _pin(pin),
        _defaultPos(defaultPos),
        _maxPos(maxPos),
        _minPos(minPos) {
    if (_minPos > _maxPos) {
      int t = _minPos;
      _minPos = _maxPos;
      _maxPos = t;
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
    int angle = (int)(x * _halfRange + _midPos);
    moveTo(angle);
  }

  void moveTo(int setPosition) {
    setPosition = clampInt(setPosition, _minPos, _maxPos);
    if (_position != setPosition) {
      _position = setPosition;
      ensureAttached();
      _servo.write(_position);
      _state = State::Holding;
      _timedDetach = false;
      debugf("MoveTo: %d", _position);
    }
  }

  void moveFor(int setPosition, unsigned long holdMs) {
    setPosition = clampInt(setPosition, _minPos, _maxPos);
    _timer = LOOP_START_NOW;
    _holdTime = holdMs;
    _timedDetach = true;

    if (_position != setPosition) {
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
  float normalizedPosition() { return ((float)_position - _midPos) / _halfRange; }

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

  static int clampInt(int p, int lo, int hi) {
    if (p < lo) return lo;
    if (p > hi) return hi;
    return p;
  }
};

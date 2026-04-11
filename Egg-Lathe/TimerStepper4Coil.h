#pragma once
#include <Arduino.h>

class TimerStepper4Coil {
public:
  enum class StepMode : uint8_t {
    Wave = 0,
    Full,
    Half
  };

  TimerStepper4Coil(
      int in1, int in2, int in3, int in4,
      StepMode mode = StepMode::Half,
      bool invertDir = false)
      : _in1(in1), _in2(in2), _in3(in3), _in4(in4),
        _stepMode(mode), _invertDir(invertDir) {}

  void begin(bool enableAtStart = true) {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);

    writeCoils(0, 0, 0, 0);

    _instance = this;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    _timer = timerBegin(1000000);  // 1 MHz timer base
    timerAttachInterrupt(_timer, &TimerStepper4Coil::onTimerStatic);
    timerAlarm(_timer, _tickUs, true, 0);
#else
    _timer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz
    timerAttachInterrupt(_timer, &TimerStepper4Coil::onTimerStatic, true);
    timerAlarmWrite(_timer, _tickUs, true);
    timerAlarmEnable(_timer);
#endif

    if (enableAtStart) {
      _enabled = true;
      applyCurrentSequence();
    } else {
      _enabled = false;
      writeCoils(0, 0, 0, 0);
    }
  }

  void setStepMode(StepMode mode) {
    portENTER_CRITICAL(&_mux);
    _stepMode = mode;
    if (_stepMode == StepMode::Half) {
      _sequenceIndex = wrapIndex(_sequenceIndex, 8);
    } else {
      _sequenceIndex = wrapIndex(_sequenceIndex, 4);
    }
    portEXIT_CRITICAL(&_mux);

    if (_enabled) {
      applyCurrentSequence();
    }
  }

  void setSpeedStepsPerSec(float stepsPerSec) {
    if (stepsPerSec < 0.0f) stepsPerSec = -stepsPerSec;

    portENTER_CRITICAL(&_mux);
    _stepsPerSec = stepsPerSec;
    if (_stepsPerSec > 0.0f) {
      _stepIntervalUs = (uint32_t)lroundf(1000000.0f / _stepsPerSec);
      if (_stepIntervalUs < _tickUs) _stepIntervalUs = _tickUs;
    }
    portEXIT_CRITICAL(&_mux);
  }

  void moveTo(long target) {
    portENTER_CRITICAL(&_mux);
    _targetPosition = target;
    if (_targetPosition != _currentPosition) {
      _moving = true;
      _enabled = true;
    }
    portEXIT_CRITICAL(&_mux);
  }

  void moveSteps(long delta) {
    portENTER_CRITICAL(&_mux);
    _targetPosition += delta;
    if (_targetPosition != _currentPosition) {
      _moving = true;
      _enabled = true;
    }
    portEXIT_CRITICAL(&_mux);
  }

  void stop(bool hold = true) {
    portENTER_CRITICAL(&_mux);
    _targetPosition = _currentPosition;
    _moving = false;
    _enabled = hold;
    portEXIT_CRITICAL(&_mux);

    if (!hold) {
      writeCoils(0, 0, 0, 0);
    } else {
      applyCurrentSequence();
    }
  }

  long currentPosition() const {
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    long v = _currentPosition;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return v;
  }

  long targetPosition() const {
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    long v = _targetPosition;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return v;
  }

  bool isMoving() const {
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    bool v = _moving;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return v;
  }

private:
  int _in1, _in2, _in3, _in4;
  StepMode _stepMode;
  bool _invertDir;

  hw_timer_t* _timer = nullptr;
  static TimerStepper4Coil* _instance;

  mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  volatile bool _enabled = false;
  volatile bool _moving = false;

  volatile long _currentPosition = 0;
  volatile long _targetPosition = 0;

  volatile float _stepsPerSec = 100.0f;
  volatile uint32_t _stepIntervalUs = 10000;  // default 100 steps/s
  volatile uint32_t _elapsedUs = 0;

  volatile int _sequenceIndex = 0;

  static constexpr uint32_t _tickUs = 100;  // ISR every 100 us

  static constexpr uint8_t HALF_SEQ[8][4] = {
      {1, 0, 0, 0},
      {1, 1, 0, 0},
      {0, 1, 0, 0},
      {0, 1, 1, 0},
      {0, 0, 1, 0},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
      {1, 0, 0, 1}
  };

  static constexpr uint8_t FULL_SEQ[4][4] = {
      {1, 1, 0, 0},
      {0, 1, 1, 0},
      {0, 0, 1, 1},
      {1, 0, 0, 1}
  };

  static constexpr uint8_t WAVE_SEQ[4][4] = {
      {1, 0, 0, 0},
      {0, 1, 0, 0},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
  };

  static void IRAM_ATTR onTimerStatic() {
    if (_instance) _instance->onTimer();
  }

  void IRAM_ATTR onTimer() {
    if (!_enabled || !_moving) return;
    if (_stepsPerSec <= 0.0f) return;

    _elapsedUs += _tickUs;
    if (_elapsedUs < _stepIntervalUs) return;
    _elapsedUs -= _stepIntervalUs;

    if (_currentPosition == _targetPosition) {
      _moving = false;
      return;
    }

    int dir = (_targetPosition > _currentPosition) ? +1 : -1;
    int logicalDir = _invertDir ? -dir : dir;

    if (_stepMode == StepMode::Half) {
      _sequenceIndex = wrapIndexISR(_sequenceIndex + logicalDir, 8);
      writeCoilsISR(
          HALF_SEQ[_sequenceIndex][0],
          HALF_SEQ[_sequenceIndex][1],
          HALF_SEQ[_sequenceIndex][2],
          HALF_SEQ[_sequenceIndex][3]);
    } else if (_stepMode == StepMode::Full) {
      _sequenceIndex = wrapIndexISR(_sequenceIndex + logicalDir, 4);
      writeCoilsISR(
          FULL_SEQ[_sequenceIndex][0],
          FULL_SEQ[_sequenceIndex][1],
          FULL_SEQ[_sequenceIndex][2],
          FULL_SEQ[_sequenceIndex][3]);
    } else {
      _sequenceIndex = wrapIndexISR(_sequenceIndex + logicalDir, 4);
      writeCoilsISR(
          WAVE_SEQ[_sequenceIndex][0],
          WAVE_SEQ[_sequenceIndex][1],
          WAVE_SEQ[_sequenceIndex][2],
          WAVE_SEQ[_sequenceIndex][3]);
    }

    _currentPosition += (dir > 0) ? 1 : -1;

    if (_currentPosition == _targetPosition) {
      _moving = false;
    }
  }

  void applyCurrentSequence() {
    if (!_enabled) return;

    if (_stepMode == StepMode::Half) {
      _sequenceIndex = wrapIndex(_sequenceIndex, 8);
      writeCoils(
          HALF_SEQ[_sequenceIndex][0],
          HALF_SEQ[_sequenceIndex][1],
          HALF_SEQ[_sequenceIndex][2],
          HALF_SEQ[_sequenceIndex][3]);
    } else if (_stepMode == StepMode::Full) {
      _sequenceIndex = wrapIndex(_sequenceIndex, 4);
      writeCoils(
          FULL_SEQ[_sequenceIndex][0],
          FULL_SEQ[_sequenceIndex][1],
          FULL_SEQ[_sequenceIndex][2],
          FULL_SEQ[_sequenceIndex][3]);
    } else {
      _sequenceIndex = wrapIndex(_sequenceIndex, 4);
      writeCoils(
          WAVE_SEQ[_sequenceIndex][0],
          WAVE_SEQ[_sequenceIndex][1],
          WAVE_SEQ[_sequenceIndex][2],
          WAVE_SEQ[_sequenceIndex][3]);
    }
  }

  void writeCoils(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    digitalWrite(_in1, a ? HIGH : LOW);
    digitalWrite(_in2, b ? HIGH : LOW);
    digitalWrite(_in3, c ? HIGH : LOW);
    digitalWrite(_in4, d ? HIGH : LOW);
  }

  void IRAM_ATTR writeCoilsISR(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    digitalWrite(_in1, a ? HIGH : LOW);
    digitalWrite(_in2, b ? HIGH : LOW);
    digitalWrite(_in3, c ? HIGH : LOW);
    digitalWrite(_in4, d ? HIGH : LOW);
  }

  static int wrapIndex(int value, int mod) {
    while (value < 0) value += mod;
    while (value >= mod) value -= mod;
    return value;
  }

  static int IRAM_ATTR wrapIndexISR(int value, int mod) {
    while (value < 0) value += mod;
    while (value >= mod) value -= mod;
    return value;
  }
};

inline TimerStepper4Coil* TimerStepper4Coil::_instance = nullptr;
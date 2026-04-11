#pragma once

#include <Arduino.h>
#include <type_traits>
#if __has_include(<Bluepad32.h>)
#include <Bluepad32.h>
#endif

#define BP32_MAX_GAMEPADS 1
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

namespace RobotControls {

template <typename T>
class Tracked {
  static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value,
                "Tracked<T> requires arithmetic or enum type");

 public:
  Tracked() : _value(), _old(), _changed(false) {}
  explicit Tracked(const T& initial) : _value(initial), _old(initial), _changed(false) {}

  bool set(const T& v1) {
    if (v1 == _value) {
      _changed = false;
      return false;
    }
    _old = _value;
    _value = v1;
    _changed = true;
    return true;
  }

  Tracked& operator=(const T& v1) {
    set(v1);
    return *this;
  }

  const T& value() const { return _value; }
  const T& old() const { return _old; }

  bool changed() const { return _changed; }
  void clearChanged() { _changed = false; }

  bool rising() const { return _changed && (_value > _old); }
  bool falling() const { return _changed && (_value < _old); }

  bool wentTo(const T& v1) const { return _changed && (_value == v1); }
  bool cameFrom(const T& v1) const { return _changed && (_old == v1); }

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

inline float NormalizeAxis(int axis, float scale = 512.0f) {
  float norm = static_cast<float>(axis) / scale;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;
  return norm;
}

struct ControlsInput {
  float axisY = 0.0f;
  float axisX = 0.0f;
  float axisRY = 0.0f;
  float axisRX = 0.0f;

  bool thumbL = false;
  bool thumbR = false;
  bool rightBumper = false;
  bool leftBumper = false;
  bool rightTrigger = false;
  bool leftTrigger = false;
  bool a = false;
  bool b = false;
  bool x = false;
  bool y = false;

  int dpad = 0;
  bool logChanges = true;
};

#if __has_include(<Bluepad32.h>)
inline ControlsInput ReadBluepadControls(ControllerPtr ctl,bool logChanges) {
  ControlsInput in;
  in.axisY = NormalizeAxis(ctl->axisY());
  in.axisX = NormalizeAxis(ctl->axisX());
  in.axisRY = NormalizeAxis(ctl->axisRY());
  in.axisRX = NormalizeAxis(ctl->axisRX());
  in.thumbL = ctl->thumbL();
  in.thumbR = ctl->thumbR();
  in.rightBumper = ctl->r1();
  in.leftBumper = ctl->l1();
  in.rightTrigger = ctl->r2();
  in.leftTrigger = ctl->l2();
  in.a = ctl->a();
  in.b = ctl->b();
  in.x = ctl->x();
  in.y = ctl->y();
  in.dpad = ctl->dpad();
  in.logChanges=logChanges;
  return in;
}
#endif

struct Controls {
  Tracked<float> axisY, axisX, axisRY, axisRX;
  Tracked<bool> thumbL, thumbR;
  Tracked<bool> rightBumper, leftBumper;
  Tracked<bool> rightTrigger, leftTrigger;
  Tracked<bool> a, b, x, y;
  Tracked<int> dpad;
  Tracked<bool> dpadUp, dpadDown;

  bool updateFrom(const ControlsInput& in) {
    bool changed = false;

    changed |= axisY.set(in.axisY);
    changed |= axisX.set(in.axisX);
    changed |= axisRY.set(in.axisRY);
    changed |= axisRX.set(in.axisRX);

    changed |= thumbL.set(in.thumbL);
    changed |= thumbR.set(in.thumbR);

    changed |= rightBumper.set(in.rightBumper);
    changed |= leftBumper.set(in.leftBumper);

    changed |= rightTrigger.set(in.rightTrigger);
    changed |= leftTrigger.set(in.leftTrigger);

    changed |= a.set(in.a);
    changed |= b.set(in.b);
    changed |= x.set(in.x);
    changed |= y.set(in.y);

    changed |= dpad.set(in.dpad);
    changed |= dpadUp.set(in.dpad == 1);
    changed |= dpadDown.set(in.dpad == 2);

    if (changed && in.logChanges) {
      Serial.printf(
          "axisY=%.3f axisX=%.3f axisRY=%.3f axisRX=%.3f | "
          "thumbL=%d thumbR=%d | "
          "LB=%d RB=%d LT=%d RT=%d | "
          "A=%d B=%d X=%d Y=%d | "
          "dpad=%d (U=%d D=%d) | "
          "changed=%d\n",
          axisY.value(), axisX.value(), axisRY.value(), axisRX.value(), thumbL.value(), thumbR.value(),
          leftBumper.value(), rightBumper.value(), leftTrigger.value(), rightTrigger.value(), a.value(), b.value(),
          x.value(), y.value(), dpad.value(), dpadUp.value(), dpadDown.value(), changed);
    }

    return changed;
  }
};


void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);

      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(),
                    properties.vendor_id,
                    properties.product_id);

      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }

  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not find empty slot");
    ctl->disconnect();
    return;
  }

  BP32.enableNewBluetoothConnections(false);
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

  BP32.enableNewBluetoothConnections(true);
}

bool processControllers(Controls& controller, bool debug) {
  bool anyChanged = false;

  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    for (auto myController : myControllers) {
      if (myController && myController->isConnected() && myController->hasData()) {
        if (myController->isGamepad()) {
          anyChanged |= controller.updateFrom(ReadBluepadControls(myController, debug));
        } else {
          Serial.println("Unsupported controller");
        }
      }
    }
  }
  return anyChanged;
}

  void connectController(){
      //   put your setup code here, to run once:
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks and start scanning
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
  }
  


}  // namespace RobotControls

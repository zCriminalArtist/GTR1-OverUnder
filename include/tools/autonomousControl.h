#ifndef AUTONOMOUSCONTROL_H
#define AUTONOMOUSCONTROL_H

#include "vex.h"
#include "coefficients.h"
#include <vector>

struct Gain {
      double* k = nullptr;
      double* actual = nullptr;
      double* target = nullptr;

      Gain(double& _k, double& _actual, double& _target) : k(&_k), actual(&_actual), target(&_target) {}
      Gain(double& _actual, double& _target) : actual(&_actual), target(&_target) {}
      Gain(double& _k) : k(&_k) {}

      double* getGain() { return k; }
    };
    
struct System {
  enum systemType {
    PID = 1,
    FEEDFORWARD = 2,
    TBH = 3
  };
  std::vector<Gain> gains;
  systemType type;
  System(systemType _type, std::vector<Gain> _gains) : type(_type), gains(_gains) {} 
};

class AutonomousControl {
  public:
    AutonomousControl() {}
    AutonomousControl(CoefficientLUT& pidGains) : paths(&pidGains) {}

    void monitor(std::vector<System> systems);
    void adjustGain1(double d);
    void adjustGain2(double d);
    void adjustGain3(double d);
    void selectNextSystem();
    void selectPreviousSystem();
    void displayMenu();
    void displaySystem();
    void enableDebug() { debug = true; }
    void disableDebug() { debug = false; };
    void disable() { disabled = true; }
    void enable() { disabled = false; }
    bool isDisabled() { return disabled; }
    bool useDebugging() { return debug; }
  private:
    bool debug = false;
    CoefficientLUT* paths;
    bool disabled = false;
    std::vector<System> systems; 
    unsigned int system_select = 0;
};

void AutonomousControl::monitor(std::vector<System> systems) {
  if (!debug) return;
  if (!isDisabled()) {
    this->systems = systems;
    displayMenu();
  }
}

void AutonomousControl::adjustGain1(double d) {
  Controller1.rumble("-");
  if (!debug) return;
  if (systems.size() == 0) return;
  if (systems[system_select].gains.size() < 1) return;
  if (systems[system_select].gains[0].getGain() == nullptr) return;
  *systems[system_select].gains[0].getGain() += d;
  Controller1.Screen.setCursor(1,5);
  Controller1.Screen.print(*systems[system_select].gains[0].getGain());
}

void AutonomousControl::adjustGain2(double d) {
  Controller1.rumble("-");
  if (!debug) return;
  if (systems.size() == 0) return;
  if (systems[system_select].gains.size() < 2) return;
  if (systems[system_select].gains[1].getGain() == nullptr) return;
  *systems[system_select].gains[1].getGain() += d;
  Controller1.Screen.setCursor(2,5);
  Controller1.Screen.print(*systems[system_select].gains[1].getGain());
}

void AutonomousControl::adjustGain3(double d) {
  Controller1.rumble("-");
  if (!debug) return;
  if (systems.size() == 0) return;
  if (systems[system_select].gains.size() < 2) return;
  if (systems[system_select].gains[2].getGain() == nullptr) return;
  *systems[system_select].gains[2].getGain() += d;
  Controller1.Screen.setCursor(3,5);
  Controller1.Screen.print(*systems[system_select].gains[2].getGain());
}

void AutonomousControl::selectNextSystem() {
  if (systems.size() == system_select + 1) return;
  system_select++;
  displayMenu();
}

void AutonomousControl::selectPreviousSystem() {
  if (system_select == 0) return;
  system_select--;
  displayMenu();
}

void AutonomousControl::displaySystem() {
  if (systems.size() == 0) return;
  if (!debug) return;
  for (int i = 0; i < systems[system_select].gains.size(); i++) {
    if (systems[system_select].gains[i].actual != nullptr) {
      Controller1.Screen.setCursor(i + 1, 13);
      Controller1.Screen.print(*systems[system_select].gains[i].actual);
      Controller1.Screen.setCursor(i + 1, 18);
      Controller1.Screen.print("/");
    }
    if (systems[system_select].gains[i].target != nullptr) {
      Controller1.Screen.setCursor(i + 1, 20);
      Controller1.Screen.print(*systems[system_select].gains[i].target);
    }
  }

  // Controller1.Screen.setCursor(1,13);
  // Controller1.Screen.print(Robot.Odometer.getRbtYPos());
  // Controller1.Screen.setCursor(1,18);
  // Controller1.Screen.print("/");
  // Controller1.Screen.setCursor(2,13);
  // Controller1.Screen.print(PIDController::toDegrees(Robot.Odometer.getPosition().h));
  // Controller1.Screen.setCursor(2,19);
  // Controller1.Screen.print("/");
  // Controller1.Screen.setCursor(1,20);
  // Controller1.Screen.print(Robot.autonomousControl.desiredVal1);
  // Controller1.Screen.setCursor(2,21);
  // Controller1.Screen.print(Robot.autonomousControl.desiredVal2);
}

void AutonomousControl::displayMenu() {
  // Controller1.rumble("-.");
  Controller1.Screen.clearScreen();
  if (systems.size() != 0) {
    if (systems[system_select].type & System::systemType::FEEDFORWARD) {
      for (int i = 0; i < systems[system_select].gains.size(); i++) {
        Controller1.Screen.setCursor(i + 1, 1);
        if (i == 0) {
          Controller1.Screen.print("kV:");
        } else if (i == 1) {
          Controller1.Screen.print("kA:");
        }
        Controller1.Screen.setCursor(i + 1, 5);
        if (systems[system_select].gains[i].getGain() != nullptr) Controller1.Screen.print(*systems[system_select].gains[i].getGain());
      }
    } else {
      for (int i = 0; i < systems[system_select].gains.size(); i++) {
        Controller1.Screen.setCursor(i + 1, 1);
        if (i == 0) {
          Controller1.Screen.print("kP:");
        } else if (i == 1) {
          Controller1.Screen.print("kI:");
        } else if (i == 2) {
          Controller1.Screen.print("kD:");
        }
        Controller1.Screen.setCursor(i + 1, 5);
        if (systems[system_select].gains[i].getGain() != nullptr) Controller1.Screen.print(*systems[system_select].gains[i].getGain());
      }
    }
  } else {
    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("P: ");
    // Controller1.Screen.setCursor(1,4);
    // Controller1.Screen.print(paths->get(debugPidPath2 ? pathKey2 : pathKey1).kP);
    // Controller1.Screen.setCursor(2,1);
    // Controller1.Screen.print("I: ");
    // Controller1.Screen.setCursor(2,4);
    // Controller1.Screen.print(paths->get(debugPidPath2 ? pathKey2 : pathKey1).kI);
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("D: ");
    // Controller1.Screen.setCursor(3,4);
    // Controller1.Screen.print(paths->get(debugPidPath2 ? pathKey2 : pathKey1).kD);
    // Controller1.Screen.setCursor(1,10);
    // Controller1.Screen.print("1: ");
    // Controller1.Screen.setCursor(2,10);
    // Controller1.Screen.print("2:");
    // Controller1.Screen.setCursor(3,11);
    // Controller1.Screen.print("[1]");
  }
}

#endif AUTONOMOUSCONTROL_H
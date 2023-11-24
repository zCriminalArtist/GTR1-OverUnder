#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "vex.h"
#include <iostream>
#include <cmath>

using namespace vex;

#define PI atan(1)*4

struct Path {
  double kP = 1.0;
  double kI = 0.0;
  double kD = 0.0;

  Path(double _kP, double _kI, double _kD) : kP(_kP), kI(_kI), kD(_kD) {}
};

class PIDController {
public:
  PIDController(Path _path, double _power, bool isAngle) : path(_path), power(_power), angle(isAngle), integralCondition(error) {}
  PIDController(Path _path, double _power, bool isAngle, double& condition) : path(_path), power(_power), angle(isAngle), integralCondition(condition) {}
  double calculate(const double reference, const double desiredValue);
  double wrapAngle(double degrees);
  static double toDegrees(double radians);
  static double toRadians(double degrees);
  void setTolerance(double toleranceAmt) { tolerance = toleranceAmt; }
  void setIntegralCondition(double& d) { integralCondition = d; }
  void setMaxPower(double maxPower) {power = maxPower;}
  void setTimeout(int milliseconds) {timeout = milliseconds;}
  void setTolTimeout(int milliseconds) {tolTimeout = milliseconds;}
  void setPath(Path _path) { path = _path; }
  double getError();
  bool isActive();
  void cancel() { active = false; }
  void activate() { active = true; }
  Path& getPath() { return path; }
  void debug() { debugger = true; }
  void printDebug();
private:
  double power = 1.0;
  Path path;
  bool debugger = true;

  double error;
  double prevError = 0;
  double derivative;
  double totalError = 0;
  double lastReference;

  double& integralCondition;

  double a = 0; // a can be anything from 0 < a < 1
  double previousFilterEstimate = 0; 
  double currentFilterEstimate = 0;

  double tolerance = 1.5;

  int timeout = 40000;
  int timeoutCounter = 0;
  int tolTimeout = 200;
  int tolTimeoutCounter = 0;

  vex::timer t;

  bool angle = false;
  bool active = true;
};

double PIDController::calculate(const double reference, const double desiredValue) {
  error = angle ? wrapAngle(desiredValue - reference) : desiredValue - reference;
  
  currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * (error - prevError); 
  previousFilterEstimate = currentFilterEstimate;

  derivative = (currentFilterEstimate / (t.time(timeUnits::sec)));
  totalError += std::abs(integralCondition) < 5 ? (error * t.time(timeUnits::sec)) : 0;
  // std::cout << integralCondition << std::endl;

  totalError = std::abs(totalError) > 13 ? copysign(1.0, totalError) * 13 : totalError;
  printDebug();

  double output = (error * path.kP) + (derivative * path.kD) + (totalError * path.kI);
  if (output > power*13) output = power*13;
  if (output < -power*13) output = -power*13;

  if (fabs(error) < tolerance) {
    if (tolTimeoutCounter >= tolTimeout) {
      tolTimeoutCounter = 0;
      active = false;
    } else {
      tolTimeoutCounter++;
    }
  } else {
    tolTimeoutCounter = 0;
  }
  if (timeoutCounter >= timeout) {
    timeoutCounter = 0;
    active = false;
  } else {
    timeoutCounter++;
  }

  prevError = error;
  lastReference = reference;
  t.reset();
  vex::task::sleep(5);

  return output;
}

double PIDController::toDegrees(double radians) {
  return radians * (180 / (PI));
}

double PIDController::toRadians(double degrees) {
  return degrees * ((PI) / 180);
}

double PIDController::wrapAngle(double degrees) {
  while (degrees > 180) {
    degrees -= 360;
  }  
  while (degrees < -180) {
    degrees += 360;
  }
  return degrees;
}

double PIDController::getError() {
  return error;
}

bool PIDController::isActive() {
  return active;
}

void PIDController::printDebug() {
  if (!debugger) return;
  // std::cout << "PORPORTION: " << error << " * " << path.kP << " = " << (error * path.kP) << std::endl;
  // std::cout << "DERIVITAVE: " << error << " - " << prevError << " = " << derivative << " * " << path.kD << " = " << (derivative * path.kD) << std::endl;
  // std::cout << "INTEGRAL: " << error * 0.05 << " + " << totalError << " = " << totalError << " * " << path.kI << " = " << (totalError * path.kI) << std::endl;
  // std::cout << (std::abs(error) < 5) << std::endl;
  // std::cout << "(" << error << " * " << path.kP << ") + (" << totalError << " * " << path.kI << ") + (" << derivative << " * " << path.kD << ") = " << (error * path.kP) + (derivative * path.kD) + (totalError * path.kI) << std::endl;
}

#endif PIDCONTROLLER_H
#pragma once

#include "vex.h"

using namespace vex;

class TBHController {
  public:
    TBHController(double gain, double maxRpm) : k(gain), kMaxRpm(maxRpm) {}
    double calculate(double actualRpm);
    void setTargetVelocity(double newRpm, bool b);
  private:
    bool isPositive(double input);
    double clamp(double input);

    double motorPower = 0;
    double k = 1E-5;
    double prevError = 10;
    double tbh = 0;
    double targetRpm = 0;
    double kMaxRpm = 3600;
};

double TBHController::calculate(double actualRpm) {
  double error = targetRpm - actualRpm;
  motorPower += k * error;
  motorPower = clamp(motorPower);
  // If the error has changed in sign since the last processing
  if (isPositive(prevError) != isPositive(error)) {
    motorPower = 0.5 * (motorPower + tbh);
    tbh = motorPower;

    prevError = error;
  }
  return motorPower;
}

void TBHController::setTargetVelocity(double newRpm, bool b) {  
  // Set up values for optimized spinup to the target
  if (targetRpm < newRpm) {
    prevError = 1;
  } else if (targetRpm > newRpm) {
    prevError = -1;
  }
  if (b) tbh = (2 * (newRpm / kMaxRpm)) - 1;
  targetRpm = newRpm;
}

double TBHController::clamp(double input) {
  if (input > 13) {
    return 13;
  }
  if (input < -13) {
    return -13;
  }
  return input;
}

bool TBHController::isPositive(double input) {
  return input > 0;
}
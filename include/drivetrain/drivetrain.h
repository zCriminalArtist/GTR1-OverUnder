#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "vex.h"
#include "motionPath.h"
#include "tools/autonomousControl.h"
#include <iostream>
#include <vector>
#include <cmath>

#define DRIVE_WHEEL_DIAMETER 3.25

using namespace std;

class drivetrainControl {
  public:
    drivetrainControl(CoefficientLUT& _pidGains, odometry& odom, AutonomousControl& _control) : pidGains(_pidGains), Odometer(odom), Control(_control) {
      // ---- Drivetrain Forward ----
      pidGains.add(1, Path(0.45, 0.80, 0.0)); // Drive
      pidGains.add(2, Path(0.3, 0.1, 0.001)); // Turn
      // ---- Drivetrain Turn -------
      pidGains.add(3, Path(0.45, 0.15, 0.001)); //Turn - Drive
      pidGains.add(4, Path(0.3, 0.55, 0.02)); //Turn - Turn
      // ---- Drivetrain to (x,y,h) -------
      pidGains.add(5, Path(0.2, 0.80, 0.0)); //Turn - Drive
      pidGains.add(6, Path(0.3, 0.75, 0.02)); //Turn - Turn
      // ---- Pure Pursuit -------
      pidGains.add(7, Path(0.02, 0.0, 0.0));
      // ----- Flywheel ----------
      pidGains.add(8, Path(0.0, 0.0, 0.0));
    } 
    void forward(double inches);
    void turn(double degrees);
    void to(double x, double y);
    void to(double x, double y, double degrees);
    void follow(MotionPath& path);
    vector<MotionPath>& getMotionPaths() { return pathsFollowed; }
    void setPower(double pwrAmt) { power = pwrAmt;}
    void setTimeout(int milliseconds) { timeout = milliseconds;}
    void setTolTimeout(int milliseconds) { tolTimeout = milliseconds; }
    void setTolerance(double toleranceAmt) { tolerance = toleranceAmt; }
    void stop();
    void reverse(bool b) { isReversed = Control.isDisabled() ? isReversed : b; }
    double velocityToRPM(const double velocity) { return (velocity * 60)/(PI * WHEEL_DIAMETER); }
    double RPMtoVelocity(const double rpm) { return rpm/60 * PI * WHEEL_DIAMETER; }
    CoefficientLUT& getPIDGains() { return pidGains; }
  private:
    double getDistance(double desiredX, double desiredY, double referenceX, double referenceY);
    CoefficientLUT& pidGains;
    odometry& Odometer;
    AutonomousControl& Control;
    vector<MotionPath> pathsFollowed;
    double power = 1.0;
    int timeout = 3000;
    int timeoutCounter = 0;
    double tolerance;
    int tolTimeout = 10;
    int tolTimeoutCounter = 0;
    bool cancel = false;
    bool isReversed = false;

    double kV = 0.2;
    double kA = 0.05;

    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double leftVelocityPower = 0.0;
    double leftAccelerationPower = 0.0;
    double leftVelocityPID = 0.0;

    double rightVelocityPower = 0.0;
    double rightAccelerationPower = 0.0;
    double rightVelocityPID = 0.0;

    double completion = 0.0;
};

void drivetrainControl::follow(MotionPath& path) {
  pathsFollowed.push_back(path);

  PIDController leftDriveVelocity = PIDController(Path(kP, kI, kD), 1.0, false, completion);
  leftDriveVelocity.setTolTimeout(200000);
  leftDriveVelocity.setTolerance(200000);
  PIDController rightDriveVelocity = PIDController(Path(kP, kI, kD), 1.0, false, completion);
  rightDriveVelocity.setTolTimeout(200000);
  rightDriveVelocity.setTolerance(200000);

  Control.monitor({ System(System::systemType::FEEDFORWARD, {
    Gain(kV, leftVelocityPower, rightVelocityPower), 
    Gain(kA, leftAccelerationPower, rightAccelerationPower),
    Gain(completion, completion) }), 
                System(System::systemType::PID, {
    Gain(kP, leftVelocityPower, rightVelocityPower),
    Gain(kI, leftAccelerationPower, leftAccelerationPower),
    Gain(kD, leftVelocityPID, rightVelocityPID) }) 
  });
  while (!Control.isDisabled()) {
    leftDriveVelocity.setPath(Path(kP, kI, kD));
    rightDriveVelocity.setPath(Path(kP, kI, kD));

    MotionPath::Pt closestPt = path.getClosestPt(Odometer.getPosition());
    MotionPath::Pt lookAheadPt = path.getLookAheadPt(Odometer.getPosition());

    double curvature = path.calculateCurvature(Odometer.getPosition(), lookAheadPt);

    auto [leftTargetVelocity, rightTargetVelocity] = path.calculateVelocity(curvature, closestPt);
    auto [leftTargetAcceleration, rightTargetAcceleration] = path.calculateAcceleration(curvature, closestPt);

    double actualLeftVelocity = RPMtoVelocity(leftRotation.velocity(velocityUnits::rpm)*PPR500);
    double actualRightVelocity = RPMtoVelocity(rightRotation.velocity(velocityUnits::rpm)*PPR500);

    leftVelocityPower = kV * leftTargetVelocity;
    leftAccelerationPower = kA * leftTargetAcceleration;
    leftVelocityPID = leftDriveVelocity.calculate(actualLeftVelocity, leftTargetVelocity);

    rightVelocityPower = kV * rightTargetVelocity;
    rightAccelerationPower = kA * rightTargetAcceleration;
    rightVelocityPID = rightDriveVelocity.calculate(actualRightVelocity, rightTargetVelocity);;

    double leftPower = leftVelocityPower + leftAccelerationPower + leftVelocityPID;
    double rightPower = rightVelocityPower + rightAccelerationPower + rightVelocityPID;

    LeftDrive.spin(vex::reverse, leftPower, volt);
    RightDrive.spin(vex::reverse, rightPower, volt);
    
    // std::cout << completion << std::endl;
    // std::cout << kV * (leftTargetVelocity) << " + " << kA * leftTargetAcceleration << " + " <<  kP << " ( " << leftTargetVelocity << " - " << actualLeftVelocity << " ) = " << leftPower
    // << "         " << kV * (rightTargetVelocity) << " + " << kA * rightTargetAcceleration << " + " <<  kP << " ( " << rightTargetVelocity << " - " << actualRightVelocity << " ) = " << rightPower  << std::endl;

    completion = (path.newPath.size() - 1) - path.lastClosestPtIndex;
    if (fabs(completion) < tolerance) {
      if (tolTimeoutCounter >= tolTimeout) {
        tolTimeoutCounter = 0;
        break;
      } else {
        tolTimeoutCounter++;
      }
    } else {
      tolTimeoutCounter = 0;
    }
    if (timeoutCounter >= timeout) {
      timeoutCounter = 0;
      break;
    } else {
      timeoutCounter++;
    }
    if (!completion) break;
    vex::task::sleep(10);
  }
  LeftDrive.stop(Control.isDisabled() ? hold : coast);
  RightDrive.stop(Control.isDisabled() ? hold : coast);
}

void drivetrainControl::forward(double inches) {
  double actualDistance;
  double desiredDistance = Odometer.getRbtYPos() + inches;
  double actualAngle;
  double desiredAngle = PIDController::toDegrees(Odometer.getPosition().h);
  PIDController driveControl = PIDController(pidGains.get(1), 1.0, false);
  driveControl.setMaxPower(power);
  driveControl.setTolerance(0.5);
  driveControl.setTolTimeout(tolTimeout);
  driveControl.setTimeout(timeout);
  PIDController turnControl = PIDController(pidGains.get(2), 1.0, false);
  turnControl.setTolTimeout(tolTimeout);
  turnControl.setTolerance(0.5);
  turnControl.setTimeout(timeout);

  Control.monitor( { System(System::systemType::PID, {
    Gain(pidGains.get(1).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(1).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(1).kD) }),
                    System(System::systemType::PID, {
    Gain(pidGains.get(2).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(2).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(2).kD) 
  })});
  while (driveControl.isActive() && !Control.isDisabled()) {
    actualDistance = Odometer.getRbtYPos();
    actualAngle = PIDController::toDegrees(Odometer.getPosition().h);
    double f = driveControl.calculate(actualDistance, desiredDistance);
    double t = turnControl.calculate(actualAngle, desiredAngle);
    LeftDrive.spin(vex::reverse, f + t, volt);
    RightDrive.spin(vex::reverse, f - t, volt);

    driveControl.debug();
  }
  LeftDrive.stop(Control.isDisabled() ? hold : coast);
  RightDrive.stop(Control.isDisabled() ? hold : coast);
}

void drivetrainControl::turn(double degrees) {
  double actualDistance;
  double desiredDistance = Odometer.getRbtYPos();
  double actualAngle;
  double desiredAngle = PIDController::toDegrees(Odometer.getPosition().h) + degrees;
  PIDController driveControl = PIDController(pidGains.get(3), 1.0, false);
  driveControl.setMaxPower(power);
  driveControl.setTolerance(0.5);
  driveControl.setTolTimeout(tolTimeout);
  PIDController turnControl = PIDController(pidGains.get(4), 1.0, false);
  turnControl.setTolerance(1.2);
  turnControl.setTolTimeout(50);
  turnControl.setTimeout(timeout);
  Control.monitor( { System(System::systemType::PID, {
    Gain(pidGains.get(3).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(3).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(3).kD) }),
                    System(System::systemType::PID, {
    Gain(pidGains.get(4).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(4).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(4).kD) 
  })});
  while (turnControl.isActive() && !Control.isDisabled()) {
    double f = driveControl.calculate(Odometer.getRbtYPos(), desiredDistance);
    double t = turnControl.calculate(PIDController::toDegrees(Odometer.getPosition().h), desiredAngle);
    LeftDrive.spin(vex::reverse, f + t, volt);
    RightDrive.spin(vex::reverse, f - t, volt);
    if (cancel) break; // Support for aiming
    turnControl.debug();
  }
  LeftDrive.stop(Control.isDisabled() ? hold : coast);
  RightDrive.stop(Control.isDisabled() ? hold : coast);
}

void drivetrainControl::to(double x, double y, double degrees) {
  MotionPath mp = MotionPath({ { (pathsFollowed.size() != 0 ? pathsFollowed.back().newPath.back().x : Odometer.getPosition().x) , (pathsFollowed.size() != 0 ? pathsFollowed.back().newPath.back().y : Odometer.getPosition().y) }, {x, y} });
  mp.inject();
  if (isReversed) mp.reverse();
  pathsFollowed.push_back(mp);
  double desiredDistance = 0.0;
  double actualDistance;
  double desiredAngle;
  double actualAngle;
  PIDController distanceControl = PIDController(pidGains.get(5), 1.0, false);
  distanceControl.setTolTimeout(5);
  distanceControl.setTolerance(1.5);
  PIDController angleControl = PIDController(pidGains.get(6), 1.0, true);
  angleControl.setTolTimeout(50);
  angleControl.setTolerance(1.5);

  Control.monitor( { System(System::systemType::PID, {
    Gain(pidGains.get(5).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(5).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(5).kD) }),
                    System(System::systemType::PID, {
    Gain(pidGains.get(6).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(6).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(6).kD) 
  })});
  while (!Control.isDisabled()) {
    double xError = x - Odometer.getPosition().x;
    double yError = y - Odometer.getPosition().y;
    desiredAngle = PIDController::toDegrees(atan2(xError, yError)) + (isReversed ? 180 : 0);
    actualDistance = hypot(xError, yError);
    actualAngle = PIDController::toDegrees(Odometer.getPosition().h);
    double f;
    double t;
    if (distanceControl.isActive()) {
      f = distanceControl.calculate(0, actualDistance);
      t = angleControl.calculate(actualAngle, desiredAngle);

      std::cout << "DRIVING" << std::endl;
      std::cout << " " << actualDistance << " - " << 0 << " = " << distanceControl.getError() << std::endl;
      std::cout << "TURNING" << std::endl;
      std::cout << " ( arctan(" << xError << "/" << yError << " ) = " << desiredAngle << ") - " << actualAngle << " = " << angleControl.getError() << std::endl;
    } else {
      desiredAngle = degrees;
      angleControl.activate();
      f = 0;
      t = angleControl.calculate(actualAngle, desiredAngle);
      
      angleControl.debug();
      if (!angleControl.isActive()) break;
    }
    f *= cos(PIDController::toRadians(std::abs(angleControl.getError()) > 90 ? 90 : angleControl.getError()));
    LeftDrive.spin(vex::reverse, (isReversed ? -1 : 1 ) * f + t, volt);
    RightDrive.spin(vex::reverse, (isReversed ? -1 : 1 ) * f - t, volt);
  }
  LeftDrive.stop(Control.isDisabled() ? hold : coast);
  RightDrive.stop(Control.isDisabled() ? hold : coast);
  std::cout << "DONE" << std::endl;
}

void drivetrainControl::to(double x, double y) {
  MotionPath mp = MotionPath({ { (pathsFollowed.size() != 0 ? pathsFollowed.back().newPath.back().x : Odometer.getPosition().x) , (pathsFollowed.size() != 0 ? pathsFollowed.back().newPath.back().y : Odometer.getPosition().y) }, {x, y} });
  mp.inject();
  if (isReversed) mp.reverse();
  pathsFollowed.push_back(mp);
  double desiredDistance = 0.0;
  double actualDistance;
  double desiredAngle;
  double actualAngle;
  PIDController distanceControl = PIDController(pidGains.get(5), 1.0, false);
  distanceControl.setTolTimeout(5);
  distanceControl.setTolerance(1.5);
  PIDController angleControl = PIDController(pidGains.get(6), 1.0, true);
  angleControl.setTolTimeout(100);
  angleControl.setTolerance(3.5);

  Control.monitor( { System(System::systemType::PID, {
    Gain(pidGains.get(5).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(5).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(5).kD) }),
                    System(System::systemType::PID, {
    Gain(pidGains.get(6).kP, actualDistance, desiredDistance),
    Gain(pidGains.get(6).kI, actualAngle, desiredAngle), 
    Gain(pidGains.get(6).kD) 
  })});
  while (distanceControl.isActive() && !Control.isDisabled()) {
    double xError = x - Odometer.getPosition().x;
    double yError = y - Odometer.getPosition().y;
    desiredAngle = PIDController::toDegrees(atan2(xError, yError)) + (isReversed ? 180 : 0);
    actualDistance = hypot(xError, yError);
    actualAngle = PIDController::toDegrees(Odometer.getPosition().h);
    double f = distanceControl.calculate(0, actualDistance);
    double t = angleControl.calculate(actualAngle, desiredAngle);
    f *= cos(PIDController::toRadians(std::abs(angleControl.getError()) > 90 ? 90 : angleControl.getError()));
  
    LeftDrive.spin(vex::reverse, (isReversed ? -1 : 1 ) * f + t, volt);
    RightDrive.spin(vex::reverse, (isReversed ? -1 : 1 ) * f - t, volt);
  }
  LeftDrive.stop(Control.isDisabled() ? hold : coast);
  RightDrive.stop(Control.isDisabled() ? hold : coast);
  std::cout << "DONE" << std::endl;
}

double drivetrainControl::getDistance(double desiredX, double desiredY, double referenceX, double referenceY) {
  return sqrt(pow(desiredY - referenceY, 2) + pow(desiredX - referenceX, 2));
}

void drivetrainControl::stop() {
  cancel = true;
  wait(100, msec);
  cancel = false;
}

#endif DRIVETRAIN_H
#include "vex.h"
#include "drivetrain/drivetrain.h"
#include "program.h"
#include "tools/gifclass.h"
#include <iostream>
#include "tools/PolynomialRegression.h"
#include "control/TBHController.h"

using namespace vex;

class robot {
  public:
    odometry Odometer = odometry(imu);
    CoefficientLUT pidPaths = CoefficientLUT();
    AutonomousControl autonomousControl = AutonomousControl(pidPaths);
    drivetrainControl Drive = drivetrainControl(pidPaths, Odometer, autonomousControl);
    //Intake
    void toggleIntake(vex::directionType direction, int speed);
    int getToggleIntakeSpeed();
    void setToggleIntakeSpeed(int speed);
    void autoToggleIntake();
    //Flywheel
    void setFlywheelSpeed(unsigned int rpm) { flywheelSpeed = rpm; }
    unsigned int getFlywheelSpeed() { return flywheelSpeed; }
    void autoFlywheel(autonomousProgram* selectedAutonomous, bool& isRunning);
    void toggleAutoFlywheel() { useAutoFlywheel = !useAutoFlywheel; }
    void toggleRaiser();
    void shoot(unsigned int i);
    void aim(double degrees);
    bool isAimed() { return aimed; }
    //Drivetrain
    void updateRobotPosition() { Odometer.updateRobotPosition(); }
    void toggleDrift();
    bool drift() { return driftToggle; }
    //Leds control
    void showAllianceColor(vex::color c);
    void showNeutralColor() { leds.state(35, percent); }
    void displayVanity() { leds.state(0, percent); }
    void displayChameleonEffect(odometry* Odometer);
    void displayRobotColors() { leds.state(5, percent); }
    void flashDiskLoaded();
    bool tankDrive = true;
    //Endgame
    void launchEndgame();
    void resetRuntime() { runtime.reset(); }
    double getRuntime() { return runtime.time(seconds); }
    // Splashscreen
    void displaySplashScreen(const char* filename);
    void displayAnimatedSplashScreen(const char* filename, int x, int y);
    bool isRunningSlpashscreen() { return runningSplashScreen; }
  private:
    void showFlywheelStatus();
    vex::timer runtime;
    bool runningSplashScreen = false;
    unsigned int flywheelSpeed;
    bool useAutoFlywheel = false;
    bool aimed = true;
    bool raiserToggle = false;
    int intakeSpeed;
    bool driftToggle = false;
    bool intakeToggle = false;
};

void robot::shoot(unsigned int i) {
  pivot.close();
  vex::wait(500, msec);
  Roller.spin(vex::reverse, 60, pct);
  vex::wait(1000, msec);
  Roller.spin(vex::forward, 100, pct);
  pivot.open();
}

void robot::displayAnimatedSplashScreen(const char* filename, int x, int y) {
  runningSplashScreen = true;
  Odometer.displayPositionOnScreen = false;
  vex::Gif gif("3dsaulcover.gif", 30, 0 );
  Odometer.displayPositionOnScreen = false;
}

void robot::displaySplashScreen(const char* filename) {
  Odometer.displayPositionOnScreen = false;
  runningSplashScreen = true;
  Brain.Screen.drawImageFromFile(filename, 0, -15); 
}

void robot::launchEndgame() {
  endgame.open();
}

void robot::toggleRaiser() {
  raiserToggle = !raiserToggle;
  if (raiserToggle) {
    raiser.open();
  } else {
    raiser.close();
  }
}

// void robot::autoFlywheel(autonomousProgram* selectedAutonomous) {
//   vex::task::sleep(2000);
//   flywheelSpeed = 3150;
//   PolynomialRegression<double> flywheelKinematics;
//   flywheelKinematics.addPoint(98.0) = 2800.0;
//   flywheelKinematics.addPoint(115.0) = 2950.0;
//   flywheelKinematics.addPoint(130.0) = 3150.0;
//   flywheelKinematics.fit();
//   Flywheel.spin(vex::forward);
//   while (true) { // Down is 0 && raiserToggle = false 
//     double dist = Odometer.getDistanceFromGoal(selectedAutonomous->getStartingPosition(), selectedAutonomous->getChosenQuadrant());
//     if (useAutoFlywheel) {
//       if (raiserToggle) toggleRaiser();
//       flywheelSpeed = flywheelKinematics.calculate(dist);
//       Flywheel.spin(vex::forward, flywheelSpeed/6, rpm);
//     } else {
//       Flywheel.spin(vex::forward, flywheelSpeed/6, rpm);
//     }
//     vex::task::sleep(20);
//   }
// }

void robot::autoFlywheel(autonomousProgram* selectedAutonomous, bool& isRunning) {
  vex::task::sleep(100);
  flywheelSpeed = 3600;
  PolynomialRegression<double> flywheelKinematics;
  flywheelKinematics.addPoint(98.0) = 2800.0 + 800;
  flywheelKinematics.addPoint(115.0) = 2950.0 + 800;
  flywheelKinematics.addPoint(130.0) = 3150.0 + 800;
  flywheelKinematics.fit();
  TBHController flywheel = TBHController(1E-3, 3600);
  flywheel.setTargetVelocity(flywheelSpeed, true);
  while (true) { // Down is 0 && raiserToggle = false
    if (isRunning) showFlywheelStatus();
    double dist = Odometer.getDistanceFromGoal(selectedAutonomous->getStartingPosition(), selectedAutonomous->getChosenQuadrant());
    if (useAutoFlywheel) {
      if (raiserToggle) toggleRaiser();
      flywheelSpeed = flywheelKinematics.calculate(dist);
    }
    flywheel.setTargetVelocity(flywheelSpeed, false);
    double v = flywheel.calculate(Flywheel.velocity(rpm)*6);
    Flywheel.spin(vex::forward, v, volt);
    vex::task::sleep(20);
  }
}

void robot::showFlywheelStatus() {
  double actualRPM;
  double Error = 0.0;
  actualRPM = Flywheel.velocity(vex::rpm)*6.0;
  Error = flywheelSpeed - actualRPM;
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Flywheel");
  Controller1.Screen.setCursor(1, 18);
  Controller1.Screen.print("rpm");
  Controller1.Screen.setCursor(1, 10);
  Controller1.Screen.print(actualRPM);
  Controller1.Screen.setCursor(2, 12);
  Controller1.Screen.print((abs((actualRPM - flywheelSpeed) < 100) ? "Ready" : "        "));
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print((useAutoFlywheel ? "Auto" : "      "));
  Controller1.Screen.setCursor(3, 6);
  Controller1.Screen.print(" pct");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print(trunc((actualRPM / flywheelSpeed) * 100));
}

void robot::setToggleIntakeSpeed(int speed) {
  intakeSpeed = speed;
}

int robot::getToggleIntakeSpeed() {
  return intakeSpeed;
}

void robot::aim(double degrees) {
  aimed = false;
  Drive.turn(degrees);
  aimed = true;
}

void robot::autoToggleIntake() {
  vex::task::sleep(2000);
  while (1) { 
    // std::cout << Intake.velocity(pct) << " " << Intake.current(pct) << std::endl;
    if ((fabs(Intake.velocity(pct)) < 10) && (Intake.current(pct) > 80)) {
      Intake.spin(reverse, 50, pct);
      wait(380, msec);
      Intake.spin(vex::forward, getToggleIntakeSpeed(), pct);
      wait(1000, msec);
    }
  }
}

void robot::toggleIntake(vex::directionType direction, int speed) {
  if (!intakeToggle) {
    Intake.spin(direction, speed, pct);
    Roller.spin(vex::forward, 100, pct);
  } else {
    Intake.spin(direction, 0, pct);
    Roller.stop(coast);
  }
  robot::intakeSpeed = speed;
  robot::intakeToggle = !robot::intakeToggle;
}

void robot::toggleDrift() {
  driftToggle = !driftToggle;
}

void robot::showAllianceColor(vex::color c) {
  if (c == red) {
    leds.state(10, percent);
  } else if (c == blue) {
    leds.state(15, percent);
  }
}

void robot::displayChameleonEffect(odometry* Odometer) {
  if (Odometer->getPosition().x > 32 && Odometer->getPosition().x < 86) {
    if (!(optic.isNearObject())) showNeutralColor();
  } else {
    if (!(optic.isNearObject())) displayRobotColors();
  }
  //std::cout << Odometer->getPosition().x << std::endl;
}

void robot::flashDiskLoaded() {
  vex::task([]() {
    leds.state(30, percent);
    wait(2000, msec);
    leds.state(5, percent);
    return 0;
  });
}
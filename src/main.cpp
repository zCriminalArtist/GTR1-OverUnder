#include "vex.h"
#include "field.h"
#include "math.h"
#include <iostream>

using namespace vex;

competition Competition;
field Field;
robot Robot;
autonomous Autonomous(Robot);

void pre_auton(void) {
  vexcodeInit();
  forwardRotation.resetRotation();
  leftRotation.resetRotation();
  rightRotation.resetRotation();
  horizontalRotation.resetRotation();
  Robot.showNeutralColor();
  Robot.setToggleIntakeSpeed(100);
  Robot.setFlywheelSpeed(3000); // Default 2700

  Autonomous.selectAutonomous(Autonomous.routine1);
  Robot.autonomousControl.disable();
  Field = field(1, 1, 237, 4, Competition, Autonomous);
  vex::thread([]() { Robot.autoToggleIntake(); });
  vex::thread([]() { Field.chooseQuadrantPrompt(); });
  vex::thread([]() { Field.chooseAutonomousPrompt(); });
  vex::thread([]() { Robot.updateRobotPosition(); });
  vex::thread([]() { Field.updateRobotPositionOnField(); });
  vex::thread([]() { Robot.autoFlywheel(Autonomous.selectedAutonomous, Autonomous.isRunning()); });
  // Robot.displayAnimatedSplashScreen("3dsaulcover.gif", 30, 0);
  // Robot.displaySplashScreen("splashscreen.png");
}

void autonomous(void) {
  Robot.displayVanity();
  Robot.autonomousControl.enable();
  Autonomous.run();
}

void usercontrol(void) {
  Robot.setFlywheelSpeed(2200);
  Robot.resetRuntime();
  Robot.setToggleIntakeSpeed(100);
  LeftDrive.stop(coast);
  RightDrive.stop(coast);
  pivot.open();
  Robot.displayRobotColors();
  // Brain.Screen.drawImageFromFile("3dsaulcover.png", 0, 0);
  while (true) {
    // std::cout << Robot.getRuntime() << std::endl;
    if (!Autonomous.isRunning()) {
      // Toggle Tank Drive
      if (Controller1.ButtonX.pressing()) {
        if (Robot.getRuntime() > 60) {
          Robot.launchEndgame();
        } else {
          Robot.tankDrive = !Robot.tankDrive;
          Controller1.rumble(Robot.tankDrive ? "." : "-");
        }
        while (Controller1.ButtonX.pressing()) {}
      }
      // Toggle Shooter Raiser
      if (Controller1.ButtonY.pressing()) {
        if (!raiser.value()) { 
          raiser.open();
        } else {
          raiser.close();
        }
        while (Controller1.ButtonY.pressing()) {}
      }
      // Toggle Auto Flywheel
      if (Controller1.ButtonB.pressing()) {
        Robot.toggleAutoFlywheel();
        Robot.setFlywheelSpeed(2500);
        while (Controller1.ButtonB.pressing()) {}
      }
      // Aim
      Controller1.ButtonL2.pressed([](){
        vex::thread([](){ 
          Robot.aim(Robot.Odometer.getGoalDirection(Autonomous.getSelectedAutonomous().getStartingPosition(), Autonomous.getSelectedAutonomous().getChosenQuadrant()));
        });
        Controller1.rumble("-");
      });
      // Intake Out (Hold)
      if (Controller1.ButtonL1.pressing()) {
        Intake.spin(vex::reverse, Robot.getToggleIntakeSpeed(), pct);
      }
      Controller1.ButtonL1.released([]() {
        Intake.stop(coast);
      });
      // Roller CCW (Hold)
      if (Controller1.ButtonR1.pressing()) {
        Roller.spin(vex::forward, 100, pct);
      }
      Controller1.ButtonR1.released([]() {
        Roller.stop(coast);
      });
      // Shoot
      if (Controller1.ButtonR2.pressing()) {
        vex::thread([](){ Controller1.rumble("-"); });
      }
      Controller1.ButtonR2.pressed([]() {
        Roller.stop(coast);
        vex::thread([](){ 
          pivot.close();
          vex::wait(500, msec);
          Roller.spin(vex::reverse, 40, pct);
        });
      });
      Controller1.ButtonR2.released([]() {
        Roller.stop(coast);
        pivot.open();
      });
      if (!Robot.tankDrive) {
        //Left Joystick - Drive
        if (abs(Controller1.Axis3.value() + Controller1.Axis4.value()) < 10) {
          if (Robot.isAimed()) {
            RightDrive.stop(Robot.drift() ? hold : coast);
            LeftDrive.stop(Robot.drift() ? hold : coast);
          }
        } else {
          Robot.Drive.stop();
          // LeftDrive.spin(vex::reverse, (pow(Controller1.Axis3.value()/100.0, 3.0)*100.0 + pow(Controller1.Axis4.value()/100.0, 3.0)*100.0), pct);
          RightDrive.spin(vex::reverse, (pow(Controller1.Axis3.value()/100.0, 3.0)*100.0 - pow(Controller1.Axis4.value()/100.0, 3.0)*100.0), pct);
          Intake.spin(vex::forward, Robot.getToggleIntakeSpeed(), pct);
          vex::thread([]() {
            vex::task::sleep(10000);
            Intake.stop(coast);
          });
        }
      } else {
        // Split arcade
        if (abs(Controller1.Axis3.value()) < 20 && abs(Controller1.Axis1.value()) < 20) {
          if (Robot.isAimed()) {
            RightDrive.stop(Robot.drift() ? hold : coast);
            LeftDrive.stop(Robot.drift() ? hold : coast);
          }
        } else {
          if (abs(Controller1.Axis3.value() + Controller1.Axis1.value()) > 40) vex::thread([]() { Robot.Drive.stop(); });
          // LeftDrive.spin(vex::reverse, (pow(Controller1.Axis3.value()/100.0, 3.0)*50.0 + (((abs(Controller1.Axis2.value())) < 90) ? pow(Controller1.Axis1.value()/100.0, 3.0)*(Controller1.ButtonL2.pressing() ? 5.0 : 30.0) : 0)), vex::velocityUnits::pct);
          RightDrive.spin(vex::reverse, (pow(Controller1.Axis3.value()/100.0, 3.0)*50.0 - (((abs(Controller1.Axis2.value())) < 90) ? pow(Controller1.Axis1.value()/100.0, 3.0)*(Controller1.ButtonL2.pressing() ? 5.0 : 30.0) : 0)), vex::velocityUnits::pct);
          Intake.spin(vex::forward, Robot.getToggleIntakeSpeed(), pct);
          Roller.spin(vex::forward, 100, pct);
          vex::thread([]() {
            vex::task::sleep(10000);
            Intake.stop(coast);
            if (Roller.velocity(pct) > 50) Roller.stop(coast);
          });
        }
      }
      //Intake
      if (Controller1.ButtonRight.pressing()) {
        Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
        while (Controller1.ButtonRight.pressing()) {}
      }
      if (Controller1.ButtonLeft.pressing()) {
        Robot.toggleIntake(vex::reverse, Robot.getToggleIntakeSpeed());
        while (Controller1.ButtonRight.pressing()) {}
      }
      //Roller
      if (Controller1.ButtonUp.pressing()) {
        Roller.spin(vex::forward, 100, pct);
        vex::thread([]() { 
          wait(300, msec); 
          Roller.stop(brakeType::coast);
        });
        Robot.setFlywheelSpeed(Robot.getFlywheelSpeed() + 50);
        while (Controller1.ButtonUp.pressing()) {}
      }
      if (Controller1.ButtonDown.pressing()) {
        Roller.spin(vex::reverse, 100, pct);
        vex::thread([]() { 
          wait(300, msec); 
          Roller.stop(brakeType::coast);
        });
        Robot.setFlywheelSpeed(Robot.getFlywheelSpeed() - 50);
        while (Controller1.ButtonDown.pressing()) {}
      }
      // Run Autonomous
      if (Controller1.ButtonA.pressing()) {
        if (!Robot.Odometer.isCalibrating()) {
          Robot.autonomousControl.enable();
          vex::thread([]() {
            Autonomous.run();
          });
          LeftDrive.stop(coast);
          RightDrive.stop(coast);
          while (Controller1.ButtonA.pressing()) {}
        }
      }
    } else {
      // Autonomous - Remote PID Tuner
      if (Controller1.ButtonRight.pressing()) {
        Robot.autonomousControl.selectNextSystem();
        while (Controller1.ButtonRight.pressing()) {}
      }
      if (Controller1.ButtonLeft.pressing()) {
        Robot.autonomousControl.selectPreviousSystem();
        while (Controller1.ButtonLeft.pressing()) {}
      }
      if (Controller1.ButtonUp.pressing()) { // kP += 0.1
        Robot.autonomousControl.adjustGain1(0.05);
        while (Controller1.ButtonDown.pressing()) {}
      }
      if (Controller1.ButtonDown.pressing()) { // kP -= 0.05
        Robot.autonomousControl.adjustGain1(-0.025);
        while (Controller1.ButtonDown.pressing()) {}
      }
      if (Controller1.ButtonX.pressing()) { // kD += 0.05
        Robot.autonomousControl.adjustGain3(0.001);
        while (Controller1.ButtonX.pressing()) {}
      }
      if (Controller1.ButtonB.pressing()) { // kD -= 0.025
        Robot.autonomousControl.adjustGain3(-0.005);
        while (Controller1.ButtonB.pressing()) {}
      }
    
      if (Controller1.ButtonR1.pressing()) { // kI += 0.05
        Robot.autonomousControl.adjustGain2(0.05);
        while (Controller1.ButtonR1.pressing()) {}
      }
      if (Controller1.ButtonR2.pressing()) { // kI -= 0.05
        Robot.autonomousControl.adjustGain2(-0.05);
        while (Controller1.ButtonR2.pressing()) {}
      }
      if (Controller1.ButtonL1.pressing()) {
        Controller1.rumble("-");
        Robot.autonomousControl.disable();
        while (Controller1.ButtonL1.pressing()) {}
      }
    
      Robot.autonomousControl.displaySystem();

      // std::cout << "DRIVING " << cancel << std::endl;
      // std::cout << " " << desiredVal1 << " - " << Odometer->getRbtYPos() << " = " << pid1.getError() << std::endl;
      // std::cout << "TURNING" << std::endl;
      // std::cout << " " << "0" << " - " << PIDController::toDegrees(Odometer->getPosition().h) << " = " << pid2.getError() << std::endl;
    }
    wait(10, msec); // Sleep the task for a short amount of time to
    // prevent wasted resources.
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

#include "vex.h"
#include "program.h"
#include "robot.h"
#include "drivetrain/motionPath.h"
#include <chrono>

struct autonomous {
  autonomous(robot& _Robot) : Robot(_Robot) {}
  autonomousProgram* selectedAutonomous;
  autonomousProgram* routine1 = new autonomousProgram(new int[4]{2, 4}, Vec(-36, 3, 0), [this]() {
    Robot.Drive.setTimeout(200);
    Robot.Drive.setTolTimeout(50);
    Robot.Drive.setTolerance(10);
    if (!Robot.autonomousControl.isDisabled()) Robot.extendIntake();
    MotionPath path1 = MotionPath({ {0, 5}, {0, 19}, {25, 19}, {25, 27}, {25, 36} }).trajectory(80, 30, -20, 5.0);
    Robot.Drive.follow(path1);
    if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::reverse, Robot.getToggleIntakeSpeed());
    wait(2000, msec);
    if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::reverse, Robot.getToggleIntakeSpeed());
    MotionPath path2 = MotionPath({ {25, 30}, {25, 27}, {25, 19}, {0, 19}, {0, 5}}).trajectory(80, 30, -20, 5.0).reverse();
    Robot.Drive.follow(path2);
    if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
    MotionPath path3 = MotionPath({ {0, 5}, {0, 30}, {-7, 40}, {-7, 60} }).trajectory(20, 30, -20, 5.0);
    Robot.Drive.follow(path3);
    wait(500, msec);
    MotionPath path4 = MotionPath({ {-6, 55}, {-15, 45}, {-20, 55} }).trajectory(20, 30, -20, 5.0).reverse();
    Robot.Drive.follow(path4);
  });
  autonomousProgram* routine2 = new autonomousProgram(new int[4]{2, 4}, Vec(-36, 3, 0), [this]() {});
  autonomousProgram* routine3 = new autonomousProgram(new int[4]{2, 4}, Vec(-36, 3, 0), [this]() {});
  autonomousProgram* routine4 = new autonomousProgram(new int[4]{1, 3}, Vec(36, 3, 0), [this]() {});
  autonomousProgram* routine5 = new autonomousProgram(new int[4]{1, 3}, Vec(36, 3, 0), [this]() {});
  autonomousProgram* routine6 = new autonomousProgram(new int[4]{1, 3}, Vec(36, 3, 0), [this]() {});
  autonomousProgram* skills   = new autonomousProgram(new int[4]{1, 3}, Vec(36, 3, 0), [this]() {});
  void run() {
    running = !Robot.autonomousControl.isDisabled();
    selectedAutonomous->run();
    running = false;
  }
  void wait(double time, vex::timeUnits units) {
    if (!Robot.autonomousControl.isDisabled()) vex::wait(time, units);
  }
  void selectAutonomous(autonomousProgram* autonomousProgram) { 
    selectedAutonomous = autonomousProgram;
    Robot.Odometer.setStartingPosition(Vec(0, 0, Robot.Odometer.toRadians(getSelectedAutonomous().getStartingPosition().h)));
  }
  autonomousProgram& getSelectedAutonomous() { return *selectedAutonomous; }
  bool running = false;
  bool& isRunning() { return running; }
  robot& Robot;
};


// #include "vex.h"
// #include "program.h"
// #include "robot.h"
// #include "drivetrain/motionPath.h"
// #include <chrono>

// struct autonomous {
//   autonomous(robot& _Robot) : Robot(_Robot) {}
//   autonomousProgram* selectedAutonomous;
//   autonomousProgram* routine1 = new autonomousProgram(new int[4]{2, 4}, Vec(72 + 12, 4, 0), [this]() {
//     if (!Robot.autonomousControl.isDisabled()) Robot.displaySplashScreen("splashscreen.png");
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleAutoFlywheel();
//     Robot.Drive.setTimeout(600);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(10);
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(700, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(2);
//     wait(300, msec);
//     MotionPath path1 = MotionPath({ {0, 5}, {10, 17}, {22, 25} }).trajectory(80, 30, -20, 5.0);
//     Robot.Drive.follow(path1);
//     wait(2000, msec);
//     MotionPath path2 = MotionPath({ {18, 26}, {7, 13}, {-4, 4} }).trajectory(80, 30, -40, 3.0).reverse();
//     Robot.Drive.follow(path2);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(300, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(300, msec);
//     MotionPath path3 = MotionPath({ {-4, 5}, {-7, 15}, {14, 42}}).trajectory(80, 25, -40, 5.0);
//     Robot.Drive.follow(path3);
//     wait(1000, msec);
//     MotionPath path4 = MotionPath({ {14, 42}, {0, 30} }).trajectory(40, 25, -40, 5.0).reverse();
//     Robot.Drive.follow(path4);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(300, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
//     MotionPath path5 = MotionPath({ {0, 30}, {5, 15}, {40, 15}, {38, 0} }).trajectory(40, 25, -40, 5.0).reverse();
//     Robot.Drive.setTimeout(500);
//     Robot.Drive.setTolTimeout(50);
//     Robot.Drive.setTolerance(10);
//     Robot.Drive.follow(path5);
//     if (!Robot.autonomousControl.isDisabled()) vex::thread([]() { 
//       vex::wait(200, msec);
//       Roller.rotateFor(100, rotationUnits::deg, 100, velocityUnits::pct); 
//     });
//     Robot.Drive.setTimeout(600);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(10);
//     wait(300, msec);
//     MotionPath path6 = MotionPath({ {38, 0}, {38, 10}, {0, 8} }).trajectory(20, 40, -20, 1.0);
//     Robot.Drive.follow(path6);
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
//     MotionPath path7 = MotionPath({ {0, 8}, {-26, 10}, {-24, 46}, }).trajectory(20, 40, -20, 1.0);
//     Robot.Drive.follow(path7);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(3000, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleIntake(vex::forward, Robot.getToggleIntakeSpeed());
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleRaiser();
//     if (!Robot.autonomousControl.isDisabled()) Robot.displayAnimatedSplashScreen("3dsaulcover.gif", 30, 0);
    
//     // MotionPath path4 = MotionPath({ {5, 35}, {5, 15}, {35, 15}, {33, -6} }).trajectory(40, 25, -40, 5.0).reverse();
//     // Robot.Drive.setTimeout(250);
//     // Robot.Drive.follow(path4);
//     // Robot.Drive.setTimeout(500);
//     // Roller.spin(vex::reverse, 100, pct);
//     // wait(1400, msec);
//     // Roller.stop(brakeType::coast);
//     // MotionPath path5 = MotionPath({ {33, 0}, {-16, 32}, {2, 42} }).trajectory(80, 30, -20, 6.0); // MotionPath({{33, 0}, {18, 3}, {-9, 40}, {2, 47} }).trajectory(80, 30, -40, 5.0);
//     // Robot.Drive.follow(path5);
//     // Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     //  wait(300, msec);
//     // if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     // wait(3000, msec);
//     // if (!Robot.autonomousControl.isDisabled()) Intake.stop(coast);
//   });
//   autonomousProgram* routine2 = new autonomousProgram(new int[4]{2, 4}, Vec(72 + 12, 4, 0), [this]() {
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleAutoFlywheel();
//     Robot.Drive.setTimeout(500);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(6);
//     if (!Robot.autonomousControl.isDisabled()) Intake.spin(vex::forward, Robot.getToggleIntakeSpeed(), pct);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(500, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(2);
//     wait(300, msec);
//     Robot.Drive.to(40, 3); // 45
//     Robot.Drive.reverse(true);
//     Robot.Drive.to(36, 3, 0);
//     Robot.Drive.reverse(false);
//     Robot.Drive.setTimeout(200);
//     Robot.Drive.setTolTimeout(300);
//     Robot.Drive.setTolerance(1.4);
//     if (!Robot.autonomousControl.isDisabled()) vex::thread([]() { 
//       vex::wait(200, msec);
//       Roller.rotateFor(100, rotationUnits::deg, 100, velocityUnits::pct); 
//     });
//     Robot.Drive.forward(-7);
//     Robot.Drive.setTimeout(700);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(6);
//     Robot.Drive.to(-4, 35);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(200, msec);
//     MotionPath path1 = MotionPath({ {-12, 40}, {0, 4} }).trajectory(40, 25, -40, 5.0).reverse();
//     Robot.Drive.follow(path1);
//     MotionPath path2 = MotionPath({ {0, 7}, {-28, 8}, {-28, 48}, {-23, 42}, {-15, 39}}).trajectory(40, 20, -30, 5.0);
//     Robot.Drive.follow(path2);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(3000, msec);
//     if (!Robot.autonomousControl.isDisabled()) Intake.stop(coast);
//   });
//   autonomousProgram* routine3 = new autonomousProgram(new int[4]{2, 4}, Vec(72 + 12, 4, 0), [this]() {
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleAutoFlywheel();
//     Robot.Drive.setTimeout(700);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(4);
//     if (!Robot.autonomousControl.isDisabled()) Intake.spin(vex::forward, Robot.getToggleIntakeSpeed(), pct);
//     MotionPath path1 = MotionPath({ {0,5}, {0, 20}}).trajectory(80, 30, -80, 3.0);
//     Robot.Drive.follow(path1);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(200, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(300, msec);
//     MotionPath path2 = MotionPath({ {0, 20}, {7, 34} }).trajectory(80, 30, -80, 3.0);
//     Robot.Drive.follow(path2);
//     Robot.Drive.setTimeout(300);
//     MotionPath path3 = MotionPath({ {7, 34}, {7, 14}, {32, 14}, {33, -7}}).trajectory(80, 30, -80, 6.0).reverse();
//     Robot.Drive.follow(path3);
//     Robot.Drive.setTimeout(700);
//     if (!Robot.autonomousControl.isDisabled()) Roller.spin(vex::reverse, 100, pct);
//     wait(1000, msec); 
//     if (!Robot.autonomousControl.isDisabled()) Roller.stop(brakeType::coast);
//     MotionPath path4 = MotionPath({ {33, 0}, {-14, 34}, {2, 42} }).trajectory(80, 30, -20, 6.0);
//     Robot.Drive.follow(path4);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(200, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(500, msec);
//     MotionPath path5 = MotionPath({ {4, 42}, {-12, 40}, {0, 4} }).trajectory(40, 25, -40, 5.0).reverse();
//     Robot.Drive.follow(path5);
//     MotionPath path6 = MotionPath({ {0, 7}, {-28, 8}, {-28, 48}, {-23, 42}, {-15, 39}}).trajectory(40, 20, -30, 5.0);
//     Robot.Drive.follow(path6);
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(300, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(3000, msec);
//     if (!Robot.autonomousControl.isDisabled()) Intake.stop(coast);
//   });
//   autonomousProgram* routine4 = new autonomousProgram(new int[4]{2, 4}, Vec(72 + 12, 4, 0), [this]() {});
//   autonomousProgram* routine5 = new autonomousProgram(new int[4]{1, 3}, Vec(50, 0, 0), [this]() {});
//   autonomousProgram* routine6 = new autonomousProgram(new int[4]{1, 3}, Vec(24 + 6, 0, 0), [this]() {});
//   autonomousProgram* skills   = new autonomousProgram(new int[4]{1, 3}, Vec(24 + 6, 0, 0), [this]() {
//     vex::timer t;
//     if (!Robot.autonomousControl.isDisabled()) Robot.setFlywheelSpeed(2250);
//     if (!Robot.autonomousControl.isDisabled()) Robot.toggleRaiser();
//     Robot.Drive.setTimeout(500);
//     Robot.Drive.setTolTimeout(200);
//     Robot.Drive.setTolerance(10);
//     if (!Robot.autonomousControl.isDisabled()) Intake.spin(vex::forward, Robot.getToggleIntakeSpeed(), pct);
//     Robot.Drive.setTimeout(100);
//     if (!Robot.autonomousControl.isDisabled()) vex::thread([]() { Roller.rotateFor(180, rotationUnits::deg, 100, velocityUnits::pct); });
//     Robot.Drive.forward(-4);
//     Robot.Drive.setTimeout(500);
//     Robot.Drive.follow(MotionPath({ {0, 0}, {-18, 20}, {-18, 90} }).trajectory(80, 40, -80, 0.5));
//     Robot.Drive.turn(Robot.Odometer.getGoalDirection(getSelectedAutonomous().getStartingPosition(), getSelectedAutonomous().getChosenQuadrant()));
//     wait(300, msec);
//     if (!Robot.autonomousControl.isDisabled()) Robot.shoot(3);
//     wait(500, msec);
//     Robot.Drive.reverse(true);
//     Robot.Drive.to(-19, 24, 90);
//     Robot.Drive.reverse(false);
//     Robot.Drive.setTimeout(200);
//     if (!Robot.autonomousControl.isDisabled()) vex::thread([]() { 
//       vex::wait(500, msec);
//       Roller.rotateFor(180, rotationUnits::deg, 100, velocityUnits::pct); 
//     });
//     Robot.Drive.forward(-7);
//     Robot.Drive.setTimeout(500);
//     Robot.Drive.to(-5, 16, 45);
//     wait(3000, msec);
//     if (!Robot.autonomousControl.isDisabled()) {
//       while (1) {
//         if (t.time(timeUnits::sec) > 55) Robot.launchEndgame();
//       }     
//     }
//     if (!Robot.autonomousControl.isDisabled()) Intake.stop(coast);
//   });
//   void run() {
//     running = !Robot.autonomousControl.isDisabled();
//     selectedAutonomous->run();
//     running = false;
//   }
//   void wait(double time, vex::timeUnits units) {
//     if (!Robot.autonomousControl.isDisabled()) vex::wait(time, units);
//   }
//   void selectAutonomous(autonomousProgram* autonomousProgram) { 
//     selectedAutonomous = autonomousProgram;
//     Robot.Odometer.setStartingPosition(Vec(0, 0, getSelectedAutonomous().getStartingPosition().h));
//   }
//   autonomousProgram& getSelectedAutonomous() { return *selectedAutonomous; }
//   bool running = false;
//   bool& isRunning() { return running; }
//   robot& Robot;
// };
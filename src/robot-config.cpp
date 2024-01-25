#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

inertial imu = inertial(PORT16);

//right drive
motor rightMotorA = motor(PORT12, ratio6_1, true); //front
motor rightMotorB = motor(PORT13, ratio6_1, true); //middle
motor rightMotorC = motor(PORT14, ratio6_1, true); //middle
motor rightMotorD = motor(PORT4, ratio6_1, false); //back
motor_group RightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

//left drive
motor leftMotorA = motor(PORT19, ratio6_1, false); //front
motor leftMotorB = motor(PORT18, ratio6_1, false); //middle
motor leftMotorC = motor(PORT17, ratio6_1, false); //middle
motor leftMotorD = motor(PORT20, ratio6_1, true); //back
motor_group LeftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);

//intake
motor Intake = motor(PORT1, ratio6_1, true);
optical intakeOptic = optical(PORT5, false);

//flywheel
motor Catapult = motor(PORT11, ratio36_1, false);
rotation catapultRotation = rotation(PORT3);
optical catapultOptic = optical(PORT6, false);


motor flywheel1 = motor(PORT8, ratio6_1, true); //top
motor flywheel2 = motor(PORT6, ratio6_1, true); //bottom
motor_group Flywheel = motor_group(flywheel1, flywheel2);

//roller
motor Roller = motor(PORT7, ratio36_1, false);

triport Expander1 = triport(PORT20);
//tracer wheels
encoder leftRotation = encoder(Expander1.C);
encoder rightRotation = encoder(Expander1.D);
encoder horizontalRotation = encoder(Expander1.E);
encoder forwardRotation = encoder(Brain.ThreeWirePort.F);
pwm_out leds = pwm_out(Brain.ThreeWirePort.E);
pneumatics pivot = pneumatics(Brain.ThreeWirePort.H);
pneumatics raiser = pneumatics(Brain.ThreeWirePort.G);
pneumatics endgame = pneumatics(Brain.ThreeWirePort.F);

pneumatics intakeExtend = pneumatics(Brain.ThreeWirePort.D);
pneumatics intakeRetract = pneumatics(Brain.ThreeWirePort.C);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
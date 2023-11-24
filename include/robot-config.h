using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern inertial imu;
extern controller Controller1;
extern motor_group LeftDrive;
extern motor flywheel1;
extern motor flywheel2;
extern motor rightMotorB;
extern motor leftMotorB;
extern motor rightMotorC;
extern motor leftMotorC;
extern motor_group RightDrive;
extern motor Intake;
extern motor Roller;
extern motor_group Flywheel;
extern pneumatics pivot;
extern pneumatics raiser;
extern pneumatics endgame;

extern encoder forwardRotation;
extern encoder leftRotation;
extern encoder rightRotation;
extern encoder horizontalRotation;
extern pwm_out leds;
extern optical optic;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
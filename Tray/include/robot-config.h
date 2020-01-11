using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial gyroacc;
extern controller Controller1;
extern motor tilter;
extern motor lift;
extern motor rightIntake;
extern motor leftIntake;
extern motor rightFront;
extern motor rightBack;
extern motor leftFront;
extern motor leftBack;
extern limit liftLimit;
extern limit tilterLimit;
extern line lineCheck;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
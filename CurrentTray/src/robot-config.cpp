#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor tilter = motor(PORT9, ratio18_1, false);
motor lift = motor(PORT7, ratio18_1, true);
motor rightIntake = motor(PORT4, ratio18_1, true);
motor leftIntake = motor(PORT1, ratio18_1, false);
motor rightFront = motor(PORT19, ratio18_1, false);
motor rightBack = motor(PORT12, ratio18_1, false);
motor leftFront = motor(PORT17, ratio18_1, true);
motor leftBack = motor(PORT14, ratio18_1, true);
limit liftLimit = limit(Brain.ThreeWirePort.A);
limit tilterLimit = limit(Brain.ThreeWirePort.B);
line lineSensor = line(Brain.ThreeWirePort.E);
controller Controller1 = controller(primary);
inertial gyroacc = inertial(PORT16);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) 
{
  // nothing to initialize
  Brain.Screen.print("%s", "                          Hold on, calibrating.");
  gyroacc.calibrate();
  task::sleep(2500);
  Controller1.Screen.print("%s", "gyro ready");
}
#include "vex.h"

using namespace vex;
using namespace std;

using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial gyroacc = inertial(PORT16);
controller Controller1 = controller(primary);
motor tilter = motor(PORT9, ratio18_1, false);
motor lift = motor(PORT7, ratio18_1, true);
motor rightIntake = motor(PORT4, ratio18_1, true);
motor leftIntake = motor(PORT1, ratio18_1, false);
motor rightFront = motor(PORT19, ratio18_1, false);//false
motor rightBack = motor(PORT12, ratio18_1, false);//false
motor leftFront = motor(PORT17, ratio18_1, true);//true
motor leftBack = motor(PORT14, ratio18_1, true);//true
limit liftLimit = limit(Brain.ThreeWirePort.A);
limit tilterLimit = limit(Brain.ThreeWirePort.B);
limit upLimit = limit(Brain.ThreeWirePort.G);
limit downLimit = limit(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
int countTask()
{
  int counter=0;
  while(true)
  {
    if(upLimit.pressing())
    {
      counter++;
      counter%=4;
      Controller1.Screen.clearLine();
      if(counter==0)
        Controller1.Screen.print("%s", "red small");
      else if(counter==1)
        Controller1.Screen.print("%s", "red large");
      else if(counter==2)
        Controller1.Screen.print("%s", "blue small");
      else if(counter==3)
        Controller1.Screen.print("%s", "blue large");
      else
        Controller1.Screen.print("%s", "uhh go up");
    }
    if(downLimit.pressing())
    {
      counter--;
      counter%=4;
      Controller1.Screen.clearLine();
      if(counter==0)
        Controller1.Screen.print("%s", "red small");
      else if(counter==1)
        Controller1.Screen.print("%s", "red large");
      else if(counter==2)
        Controller1.Screen.print("%s", "blue small");
      else if(counter==3)
        Controller1.Screen.print("%s", "blue large");
      else
        Controller1.Screen.print("%s", "uhh go up");
    }
    vex::task::sleep(100);
  }
}
void vexcodeInit( void ) {
  // nothing to initialize
  gyroacc.calibrate();
  task::sleep(2000);
  Brain.Screen.print("%s", "gyro ready");
  vex::task p(countTask);
}
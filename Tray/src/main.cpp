/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       yvette                                                    */
/*    Created:      Wed Jan 08 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// gyroacc              inertial      10              
// Controller1          controller                    
// tilter               motor         9               
// lift                 motor         7               
// rightIntake          motor         4               
// leftIntake           motor         3               
// rightFront           motor         19              
// rightBack            motor         12              
// leftFront            motor         17              
// leftBack             motor         14              
// liftLimit            limit         A               
// tilterLimit          limit         B               
// lineCheck            line          E               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;

int dropOff=4600;
int armLift=2900;
bool driving=false;
bool trayUp=false;

int printing()
{
 while(true)
 {
   Controller1.Screen.print("%d",gyroacc.heading(degrees));
   task::sleep(200);
 }
}
void autonomous( void ) 
{
 gyroacc.calibrate();
 task::sleep(2000);
 // ..........................................................................
 // https://www.vexforum.com/t/vexcode-motor-groups-and-drivetrain-example/69161
 // left axel to right axel
 // front axel to back axel
 // ..........................................................................
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 10.25, distanceUnits::in, 1);
  //vex::task d(printing);
  Drivetrain.turnFor(turnType::left, 90, rotationUnits::deg, 20, velocityUnits::pct);

  //Drivetrain.driveFor(directionType::rev, 17.5, distanceUnits::in, 25, velocityUnits::pct,1);
  Controller1.Screen.print("%s","done");
 }

int driveTask()
{
 while(true)
 {
   double lpower=Controller1.Axis3.value();
   double rpower=Controller1.Axis2.value();
 
   lpower=lpower * 12.0 / 127;
   rpower=rpower * 12.0 / 127;
 
   if(abs(lpower)>0.3 && abs(rpower)>0.3)
     driving=true;
   else
     driving=false;
 
   rightFront.spin(vex::directionType::fwd, rpower, vex::voltageUnits::volt);
   rightBack.spin(vex::directionType::fwd, rpower, vex::voltageUnits::volt);
   leftFront.spin(vex::directionType::fwd, lpower, vex::voltageUnits::volt);
   leftBack.spin(vex::directionType::fwd, lpower, vex::voltageUnits::volt);
 
   vex::task::sleep(2);
 }
 return 0;
}
bool armsDown=false;
int down=0;
int printTask()
{

 while(true)
 {
   Controller1.Screen.clearLine();
   if(liftLimit.pressing())
   {
     armsDown=true;
   }
   else
   {
     armsDown=false;
   }
   //tray down
   if(tilterLimit.pressing())
   {
    Controller1.Screen.print("%s","T");
    down=1;
   }
   //tray stacking
   else if(!tilterLimit.pressing() && armsDown==true)
   {
    Controller1.Screen.print("%s","F");
    down=0;
   }
   //tray down but arms up
   else
   {
     down=2;
   }
   task::sleep(200);
 }
}
bool spinning=false;
 
void lineWork(int val)
{
 bool high=false;
 Brain.resetTimer();
 while(lineCheck.value(analogUnits::range12bit)>2850 && Brain.timer(timeUnits::sec)<2)
 {
   high=true;
   rightIntake.spin(directionType::rev, 100, percentUnits::pct);
   leftIntake.spin(directionType::rev, 100, percentUnits::pct);
 }
 if(high==true)
 {
   rightIntake.spin(directionType::rev, 100, percentUnits::pct);
   leftIntake.spin(directionType::rev, 100, percentUnits::pct);
   vex::task::sleep(val);
   rightIntake.stop();
   leftIntake.stop();
 }
}
int fullTask()
{
 while(true)
 {
   if(Controller1.ButtonX.pressing())
   {
     lift.stop();
     tilter.stop();
     leftIntake.stop();
     rightIntake.stop();
     while(Controller1.ButtonX.pressing())
     {
       task::sleep(10);
     }
   }
   if(Controller1.ButtonLeft.pressing())
   {
     if(!liftLimit.pressing())
     {
       lift.spin(directionType::rev, 100, percentUnits::pct);
       task::sleep(300);
     }
     while(!liftLimit.pressing() || !tilterLimit.pressing())
     {
       if(!liftLimit.pressing())
         lift.spin(directionType::rev, 100, percentUnits::pct);
       else
         lift.stop();
       if(!tilterLimit.pressing())
         tilter.spin(directionType::rev, 100, percentUnits::pct);
       else
         tilter.stop();
     }
   }
   //intake
   if(Controller1.ButtonR1.pressing())
   {
     leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
     rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
   }
   //outtake
   else if(Controller1.ButtonR2.pressing() || (down==0 && driving))
   {
     spinning=true;
     int speed=0;
     if(Controller1.ButtonR2.pressing())
       speed=100;
 
     if(down==0 && driving)
       speed=25;
 
     leftIntake.spin(directionType::rev, speed, percentUnits::pct);
     rightIntake.spin(directionType::rev, speed, percentUnits::pct);
   }
   else
   {
     spinning=false;
     rightIntake.stop();
     leftIntake.stop();
   }
   //arms up
   if(Controller1.ButtonL1.pressing())
   {
     //lineWork(100);
     tilter.startSpinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
     lift.spinTo(armLift, rotationUnits::raw, 100, velocityUnits::pct);
     while(Controller1.ButtonL1.pressing())
     {
       task::sleep(20);
     }
   }
   //arms down
   else if(Controller1.ButtonL2.pressing() && !liftLimit.pressing())
   {
     lift.spin(directionType::rev, 60, percentUnits::pct);
   }
   else
   {
     lift.stop();
   }
   //tilt up
   if(Controller1.ButtonA.pressing())
   {
     //lineWork(140);
    
     double speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
     while(tilter.rotation(rotationUnits::raw)<dropOff)
     {
       speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
       tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
       task::sleep(100);
     }
     tilter.stop();
     trayUp=true;
     while(Controller1.ButtonA.pressing())
     {
       task::sleep(50);
     }
   }
   //tilt down
   else if(Controller1.ButtonB.pressing())
   {
     tilter.setBrake(brakeType::hold);
     while(!tilterLimit.pressing())
     {
       tilter.spin(directionType::rev, 100, percentUnits::pct);
       task::sleep(10);
     }
     tilter.stop();
     trayUp=false;
     tilter.resetRotation();
     while(Controller1.ButtonA.pressing())
     {
       task::sleep(50);
     }
   }
   task::sleep(100);
 }
}
void usercontrol( void )
{
 rightIntake.setBrake(brakeType::hold);
 leftIntake.setBrake(brakeType::hold);
 lift.setBrake(brakeType::hold);
 tilter.setBrake(brakeType::hold);
 
 lift.startSpinTo(2300, rotationUnits::raw, 100, velocityUnits::pct);
 leftIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
 rightIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
 tilter.spinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
 vex::task::sleep(500);
 leftIntake.startSpinTo(-5000, rotationUnits::raw, 100, velocityUnits::pct);
 rightIntake.startSpinTo(-5000, rotationUnits::raw, 100, velocityUnits::pct);
 if(!liftLimit.pressing())
     {
       lift.spin(directionType::rev, 100, percentUnits::pct);
       task::sleep(300);
     }
     while(!liftLimit.pressing() || !tilterLimit.pressing())
     {
       if(!liftLimit.pressing())
         lift.spin(directionType::rev, 100, percentUnits::pct);
       else
         lift.stop();
       if(!tilterLimit.pressing())
         tilter.spin(directionType::rev, 100, percentUnits::pct);
       else
         tilter.stop();
     }

 vex::task d(driveTask);
 vex::task f(fullTask);
 vex::task p(printTask);
 while (1)
 {
   vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
 }
}
int main() {
   //Set up callbacks for autonomous and driver control periods.
   Competition.autonomous( autonomous );
   Competition.drivercontrol( usercontrol );
  
   //Run the pre-autonomous function.
   vexcodeInit();
     
   //Prevent main from exiting with an infinite loop.                       
   while(1) {
     vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
   }   
     
}
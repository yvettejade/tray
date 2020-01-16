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
int armLiftHigh=2850;
int armLiftLow=2075;
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
void leftTurn(double val)
{
   rightFront.resetRotation();
   leftFront.resetRotation();
   rightBack.resetRotation();
   leftBack.resetRotation();
   rightFront.setBrake(vex::brakeType::brake);
   leftFront.setBrake(vex::brakeType::brake);
   rightBack.setBrake(vex::brakeType::brake);
   leftBack.setBrake(vex::brakeType::brake);
 
   double pConstant=0.15;
   double iConstant=0.001;
   double dConstant=0.0002;
   double error=0;
   double targetValue=gyroacc.rotation(vex::rotationUnits::deg)-(val*79);
   double speed=0;
   double integral=0;
   double derivative=0;
   double prevError=0;
   bool first=true;

   Brain.resetTimer();
   
   while(error>10||first==true)
   {
     if(Brain.timer(timeUnits::sec)>=2)
      return;
     first=false;
     double avg=gyroacc.rotation(vex::rotationUnits::deg);
    
     error=targetValue-avg;
     error=abs(error);
    
     integral+=error;
     if(error>10){
       integral=0;
     }
     derivative=abs(error-prevError);
     prevError=error;
     //Controller1.Screen.print("%f",error);
     speed=abs(pConstant*error)+abs(iConstant*integral);
     //Controller1.Screen.clearLine();
     //Controller1.Screen.print("%f",speed);
     //1*1000+1*100+1*1
     rightFront.spin(vex::directionType::fwd, abs(speed), vex::voltageUnits::volt);
     leftFront.spin(vex::directionType::rev, abs(speed), vex::voltageUnits::volt);
     rightBack.spin(vex::directionType::fwd, abs(speed), vex::voltageUnits::volt);
     leftBack.spin(vex::directionType::rev, abs(speed), vex::voltageUnits::volt);
     vex::task::sleep(10);
   }
   Controller1.Screen.print("%s","Done");
   rightFront.stop();
   leftFront.stop();
   rightBack.stop();
   leftBack.stop();
   return;
}
void rightTurn(double val)
{
   rightFront.resetRotation();
   leftFront.resetRotation();
   rightBack.resetRotation();
   leftBack.resetRotation();
   rightFront.setBrake(vex::brakeType::brake);
   leftFront.setBrake(vex::brakeType::brake);
   rightBack.setBrake(vex::brakeType::brake);
   leftBack.setBrake(vex::brakeType::brake);
 
   double pConstant=0.08;
   double iConstant=0.001;
   double dConstant=0.0002;
   double error=0;
   double targetValue=gyroacc.rotation(vex::rotationUnits::deg)+(val*74);
   double speed=0;
   double integral=0;
   double derivative=0;
   double prevError=0;
   bool first=true;

   Brain.resetTimer();

   while(error>10||first==true)
   {
     if(Brain.timer(timeUnits::sec)>=2)
      return;
     first=false;
     double avg=gyroacc.rotation(vex::rotationUnits::deg);
     //Controller1.Screen.clearLine();
     //Controller1.Screen.print("%f",avg);
     error=targetValue-avg;
     integral+=error;
     if(error<10){
       integral=0;
     }
     derivative=error-prevError;
     prevError=error;
     //Controller1.Screen.print("%f",error);
     speed=pConstant*error+iConstant*integral;
     //1*1000+1*100+1*1
     rightFront.spin(vex::directionType::rev, speed, vex::voltageUnits::volt);
     leftFront.spin(vex::directionType::fwd, speed, vex::voltageUnits::volt);
     rightBack.spin(vex::directionType::rev, speed, vex::voltageUnits::volt);
     leftBack.spin(vex::directionType::fwd, speed, vex::voltageUnits::volt);
     vex::task::sleep(10);
   }
   Controller1.Screen.print("%s","Done");
   rightFront.stop();
   leftFront.stop();
   rightBack.stop();
   leftBack.stop();
   return;
}
void stack()
{
 tilter.setBrake(brakeType::hold);
 double speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
 while(tilter.rotation(rotationUnits::raw)<dropOff)
 {
   if(tilter.rotation(rotationUnits::raw)>2300)
   {
     tilter.setBrake(brakeType::coast);
   }
   speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
   tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
   task::sleep(100);
 }
 tilter.stop();
 tilter.setBrake(brakeType::hold);
 trayUp=true;
}
void deploy()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);
 
 rightIntake.setBrake(brakeType::hold);
 leftIntake.setBrake(brakeType::hold);
 lift.setBrake(brakeType::hold);
 tilter.setBrake(brakeType::coast);
 lift.startSpinTo(2300, rotationUnits::raw, 100, velocityUnits::pct);
 leftIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
 rightIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
 tilter.spinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
 vex::task::sleep(500);
 //leftIntake.startSpinTo(5000, rotationUnits::raw, 100, velocityUnits::pct);
 //rightIntake.startSpinTo(5000, rotationUnits::raw, 100, velocityUnits::pct);
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
    lift.stop();
    tilter.stop();
}
void redSmall()
{
 
}
void redLarge()
{
 }
void blueSmall()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.63), distanceUnits::in, 25, velocityUnits::pct,1);
  task::sleep(300);
  Drivetrain.driveFor(directionType::rev, (18.5*0.2), distanceUnits::in, 25, velocityUnits::pct,1);
  task::sleep(300);
  rightTurn(0.35);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.2), distanceUnits::in, 25, velocityUnits::pct,1);
  task::sleep(500);
  leftTurn(0.35);
  Drivetrain.driveFor(directionType::rev, (18.5*1), distanceUnits::in, 25, velocityUnits::pct,1);
  rightIntake.stop();
  leftIntake.stop();
  /*rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.75), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(500);
  rightIntake.stop();
  leftIntake.stop();
  task::sleep(300);
  Drivetrain.driveFor(directionType::rev, (18.5*0.9), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(300);
  rightTurn(0.5);
  task::sleep(300);
  //back up for second stack
  Drivetrain.driveFor(directionType::rev, (18.5*1.13), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(300);
  leftTurn(0.61);
  task::sleep(300);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.75), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(400);
  Drivetrain.driveFor(directionType::rev, (18.5*1), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(300);
  rightIntake.stop();
  leftIntake.stop();
  leftTurn(1.18);
  task::sleep(300);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.5), distanceUnits::in, 25, velocityUnits::pct,1);
  task::sleep(300);
  stack();
  task::sleep(500);
  Drivetrain.driveFor(directionType::rev, (18.5*0.5), distanceUnits::in, 50, velocityUnits::pct,1);
  task::sleep(300);*/
  /*rightIntake.stop();
  leftIntake.stop();
  Drivetrain.driveFor(directionType::rev, (18.5*1), distanceUnits::in, 30, velocityUnits::pct,1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  task::sleep (500);

  rightTurn(2.15);
  Drivetrain.driveFor(directionType::fwd, (18.5*.75), distanceUnits::in, 30, velocityUnits::pct,1);
*/
}
void blueLarge()
{
}
void autonomous( void )
{
// ..........................................................................
// https://www.vexforum.com/t/vexcode-motor-groups-and-drivetrain-example/69161
// left axel to right axel
// front axel to back axel
// ..........................................................................
 leftFront.setBrake(brakeType::brake);
 rightFront.setBrake(brakeType::brake);
 leftBack.setBrake(brakeType::brake);
 rightBack.setBrake(brakeType::brake);
 motor_group leftDrive(leftFront, leftBack);
 motor_group rightDrive(rightFront, rightBack);
 smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

 Drivetrain.driveFor(directionType::fwd, (18.5), distanceUnits::in, 20, velocityUnits::pct,1);
 task::sleep(300);
 deploy();
 task::sleep(300);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
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
   down=1;
  }
  //tray stacking
  else if(!tilterLimit.pressing() && armsDown==true)
  {
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
int fullTask()
{
while(true)
{
  
  if(Controller1.ButtonLeft.pressing())
  {
    tilter.setBrake(brakeType::coast);
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
  //medium tower
  if(Controller1.ButtonL1.pressing())
  {
    //lineWork(100);
    tilter.setBrake(brakeType::hold);
    tilter.startSpinTo(1500, rotationUnits::raw, 100, velocityUnits::pct);
    lift.spinTo(armLiftHigh, rotationUnits::raw, 100, velocityUnits::pct);
    while(Controller1.ButtonL1.pressing())
    {
      task::sleep(20);
    }
  }
  //low tower
  else if(Controller1.ButtonL2.pressing())
  {
    tilter.setBrake(brakeType::hold);
    tilter.startSpinTo(1500, rotationUnits::raw, 100, velocityUnits::pct);
    lift.spinTo(armLiftLow, rotationUnits::raw, 100, velocityUnits::pct);
    while(Controller1.ButtonL2.pressing())
    {
      task::sleep(20);
    }
  }
  else
  {
    lift.stop();
  }
  //tilt up
  if(Controller1.ButtonA.pressing())
  {
    //lineWork(140);
    tilter.setBrake(brakeType::hold);
    double speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
    while(tilter.rotation(rotationUnits::raw)<dropOff)
    {
      if(tilter.rotation(rotationUnits::raw)>2300)
      {
        tilter.setBrake(brakeType::coast);
      }
      speedTilter=-0.004*tilter.rotation(rotationUnits::raw)+90;
      tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
      task::sleep(100);
    }
    tilter.stop();
    tilter.setBrake(brakeType::hold);
    trayUp=true;
    while(Controller1.ButtonA.pressing())
    {
      task::sleep(50);
    }
  }
  //tilt down
  else if(Controller1.ButtonB.pressing())
  {
    tilter.setBrake(brakeType::coast);
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
tilter.setBrake(brakeType::coast);
/*lift.startSpinTo(2300, rotationUnits::raw, 100, velocityUnits::pct);
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
    lift.stop();
    tilter.stop();*/
vex::task d(driveTask);
vex::task f(fullTask);
vex::task p(printTask);

while (1)
{
  if(Controller1.ButtonX.pressing())
  {
    lift.stop();
    tilter.stop();
    rightIntake.stop();
    leftIntake.stop();
    tilter.setBrake(brakeType::coast);
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
    f.stop();
    while(Controller1.ButtonX.pressing())
      task::sleep(20);
  }
  if(Controller1.ButtonY.pressing())
  {
    vex::task f(fullTask);
    while(Controller1.ButtonY.pressing())
      task::sleep(20);
  }
  vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
}
}
int main() {
  //Run the pre-autonomous function.
  vexcodeInit();
 
  //Set up callbacks for autonomous and driver control periods.
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );
 
   
  //Prevent main from exiting with an infinite loop.                      
  while(1) {
    vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
  }  
   
}
 


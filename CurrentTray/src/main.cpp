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
// tilter               motor         9               
// lift                 motor         7               
// rightIntake          motor         4               
// leftIntake           motor         1               
// rightFront           motor         19              
// rightBack            motor         12              
// leftFront            motor         17              
// leftBack             motor         14              
// liftLimit            limit         A               
// tilterLimit          limit         B               
// lineSensor           line          F               
// Controller1          controller                    
// gyroacc              inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
 
#include "vex.h"

using namespace vex;
competition Competition;
 
int dropOff=4600;
int armLiftHigh=2850;
int armLiftLow=2225;
bool driving=false;
bool trayUp=false;
 
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
 
   double pConstant=0.18;
   double iConstant=0.001;
   double dConstant=0.0002;
   double error=0;
   double targetValue=gyroacc.rotation(vex::rotationUnits::deg)-(val*80);
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
 
   double pConstant=0.1;
   double iConstant=0.001;
   double dConstant=0.0002;
   double error=0;
   double targetValue=gyroacc.rotation(vex::rotationUnits::deg)+(val*75);
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
  //1025 20
  leftIntake.startSpinFor(directionType::rev, 1025, rotationUnits::raw, 20, velocityUnits::pct);
  rightIntake.startSpinFor(directionType::rev, 1025, rotationUnits::raw, 20, velocityUnits::pct);
  tilter.setBrake(brakeType::hold);

 double speedTilter=-0.01*tilter.rotation(rotationUnits::raw)+80;
 bool start=false;

 while(tilter.rotation(rotationUnits::raw)<dropOff)
 {
   if(tilter.rotation(rotationUnits::raw)>2300)
   {
     tilter.setBrake(brakeType::coast);
   }
   //if(start==false && tilter.rotation(rotationUnits::raw)>=3800)
   // Drivetrain.driveFor(directionType::fwd, (18.5*0.1), distanceUnits::in, 10, velocityUnits::pct,0);
   speedTilter=-0.01*tilter.rotation(rotationUnits::raw)+80;
   tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
   task::sleep(100);
 }
 tilter.stop();
 tilter.setBrake(brakeType::hold);
 trayUp=true;

  /*motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  leftIntake.startSpinFor(directionType::rev, 1025, rotationUnits::raw, 30, velocityUnits::pct);
  rightIntake.spinFor(directionType::rev, 1000, rotationUnits::raw, 30, velocityUnits::pct);
  vex::task::sleep(100);
  leftIntake.startSpinFor(directionType::fwd, 350, rotationUnits::raw, 50, velocityUnits::pct);
  rightIntake.spinFor(directionType::fwd, 350, rotationUnits::raw, 50, velocityUnits::pct);
  vex::task::sleep(400);
 tilter.setBrake(brakeType::hold);
 double speedTilter=-0.01*tilter.rotation(rotationUnits::raw)+80;
 bool start=false;
 while(tilter.rotation(rotationUnits::raw)<dropOff)
 {
   if(tilter.rotation(rotationUnits::raw)>2300)
   {
     tilter.setBrake(brakeType::coast);
   }
   //if(start==false && tilter.rotation(rotationUnits::raw)>=3800)
   // Drivetrain.driveFor(directionType::fwd, (18.5*0.1), distanceUnits::in, 10, velocityUnits::pct,0);
   speedTilter=-0.01*tilter.rotation(rotationUnits::raw)+80;
   tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
   task::sleep(100);
 }
 tilter.stop();
 tilter.setBrake(brakeType::hold);
 trayUp=true;

 task::sleep(200);
 rightIntake.spin(directionType::rev, 45, percentUnits::pct);
 leftIntake.spin(directionType::rev, 45, percentUnits::pct); 
 Drivetrain.driveFor(directionType::rev, (18.5*1), distanceUnits::in, 20, velocityUnits::pct,1);
*/
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
    tilter.spinTo(900, rotationUnits::raw, 100, velocityUnits::pct);
    leftIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    rightIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    tilter.spinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
    vex::task::sleep(700);
    leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
    rightIntake.spin(directionType::fwd, 100, percentUnits::pct);

    Brain.resetTimer();
    if(!liftLimit.pressing())
        {
          lift.spin(directionType::rev, 100, percentUnits::pct);
          task::sleep(300);
      }
      while((!liftLimit.pressing() || !tilterLimit.pressing()) && Brain.timer(timeUnits::sec)<3)
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
      rightFront.spin(directionType::rev, 100, percentUnits::pct);
      rightBack.spin(directionType::rev, 100, percentUnits::pct);
      leftFront.spin(directionType::rev, 100, percentUnits::pct);
      leftBack.spin(directionType::rev, 100, percentUnits::pct);
      task::sleep(500);
}
void deploySkills()
{
    motor_group leftDrive(leftFront, leftBack);
    motor_group rightDrive(rightFront, rightBack);
    smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);
    
    rightIntake.setBrake(brakeType::hold);
    leftIntake.setBrake(brakeType::hold);
    lift.setBrake(brakeType::hold);
    tilter.setBrake(brakeType::coast);

    lift.startSpinTo(2300, rotationUnits::raw, 100, velocityUnits::pct);
    tilter.spinTo(900, rotationUnits::raw, 100, velocityUnits::pct);
    leftIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    rightIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    tilter.spinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
    vex::task::sleep(700);
    leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
    rightIntake.spin(directionType::fwd, 100, percentUnits::pct);

    Brain.resetTimer();
    if(!liftLimit.pressing())
        {
          lift.spin(directionType::rev, 100, percentUnits::pct);
          task::sleep(300);
      }
      while((!liftLimit.pressing() || !tilterLimit.pressing()) && Brain.timer(timeUnits::sec)<3)
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
 motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.8), distanceUnits::in, 35, velocityUnits::pct,1);
  task::sleep(300);
  leftTurn(0.45);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.5), distanceUnits::in, 30, velocityUnits::pct,1);
  task::sleep(200);
  Drivetrain.driveFor(directionType::rev, (18.5*0.5), distanceUnits::in, 45, velocityUnits::pct,1);
  task::sleep(200);
  leftIntake.stop();
  rightIntake.stop();
  leftTurn(1.9);
  task::sleep(100);
  leftFront.startSpinFor(directionType::fwd, 2000, rotationUnits::raw, 50, velocityUnits::pct);
  rightFront.startSpinFor(directionType::fwd, 2000, rotationUnits::raw, 50, velocityUnits::pct);
  leftBack.startSpinFor(directionType::fwd, 2000, rotationUnits::raw, 50, velocityUnits::pct);
  rightBack.spinFor(directionType::fwd, 2000, rotationUnits::raw, 50, velocityUnits::pct);
  rightIntake.stop();
  leftIntake.stop();
  task::sleep(300);
  stack();
}
void blueLarge()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 19, distanceUnits::in, 45, velocityUnits::pct);
  task::sleep(200);

  leftTurn(1.05);

  task::sleep(200);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 8, distanceUnits::in, 45, velocityUnits::pct);
  task::sleep(500);

  leftTurn(0.4);
  rightIntake.stop();
  leftIntake.stop();

  Drivetrain.driveFor(directionType::rev, 18.5*1.3, distanceUnits::in, 60, velocityUnits::pct);
  task::sleep(100);

  leftTurn(1.2);

  task::sleep(100);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 18, distanceUnits::in, 45, velocityUnits::pct);

  task::sleep(500);
  stack();
}
void blueSmall()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.8), distanceUnits::in, 35, velocityUnits::pct,1);
  task::sleep(300);
  rightTurn(0.35);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.5), distanceUnits::in, 40, velocityUnits::pct,1);
  task::sleep(300);
  Drivetrain.driveFor(directionType::rev, (18.5*0.6), distanceUnits::in, 40, velocityUnits::pct,1);
  task::sleep(300);
  leftIntake.stop();
  rightIntake.stop();
  rightTurn(1.95);
  task::sleep(300);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.6), distanceUnits::in, 50, velocityUnits::pct,1);
  rightIntake.stop();
  leftIntake.stop();
  task::sleep(300);
  stack();
}
void redLarge()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 20, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(300);
  rightTurn(1.15);
  task::sleep(200);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 6, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(500);
  rightTurn(0.35);
  rightIntake.stop();
  leftIntake.stop();
  Drivetrain.driveFor(directionType::rev, 18.5*1.1, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(100);
  rightTurn(1);
  task::sleep(100);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 20, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(300);
  stack();
}
void other()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.4), distanceUnits::in, 30, velocityUnits::pct,1);

  task::sleep(500);

  leftFront.spin(directionType::rev, 25, percentUnits::pct);
  leftBack.spin(directionType::rev, 25, percentUnits::pct);
  rightFront.spin(directionType::rev, 25, percentUnits::pct);
  rightBack.spin(directionType::rev, 25, percentUnits::pct);
  task::sleep(1000);

  Drivetrain.driveFor(directionType::fwd, (18.5*4.5), distanceUnits::in, 30, velocityUnits::pct,1);
  task::sleep(500);
  rightTurn(0.6);
  task::sleep(500);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.35), distanceUnits::in, 30, velocityUnits::pct,1);
  vex::task::sleep(500);
  stack();
}
void fuckit()
{
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

  rightIntake.stop();
  leftIntake.stop();

  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.8), distanceUnits::in, 30, velocityUnits::pct,1);
  task::sleep(300);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  rightTurn(0.35);
  Drivetrain.driveFor(directionType::fwd, (18.5*0.5), distanceUnits::in, 33, velocityUnits::pct,1);
  task::sleep(300);
  Drivetrain.driveFor(directionType::rev, (18.5*0.5), distanceUnits::in, 40, velocityUnits::pct,1);
  task::sleep(300);
  leftTurn(0.65);
  task::sleep(300);
  rightIntake.stop();
  leftIntake.stop();
  Drivetrain.driveFor(directionType::rev, (18.5*1.5), distanceUnits::in, 60, velocityUnits::pct,1);
  task::sleep(300);
  rightTurn(0.515);
  task::sleep(300);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, (18.5*1.5), distanceUnits::in, 33, velocityUnits::pct,1);
  
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
 
 //deploy();   
 //other();
 stack();
 //redLarge();  
 //redSmall();
 //blueLarge();
 //blueSmall();        
 //fuckit();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
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
      speed=45;
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
    lift.spin(directionType::rev, 5, velocityUnits::pct);
    tilter.setBrake(brakeType::hold);
    double speedTilter=-0.013*tilter.rotation(rotationUnits::raw)+80;
    while(tilter.rotation(rotationUnits::raw)<dropOff)
    {
      if(tilter.rotation(rotationUnits::raw)>2300)
      {
        tilter.setBrake(brakeType::coast);
      }
      speedTilter=-0.01*tilter.rotation(rotationUnits::raw)+90;
      printf("%6.6f\n",speedTilter);
      tilter.spin(directionType::fwd, speedTilter, percentUnits::pct);
      task::sleep(100);
    }
    tilter.stop();
    lift.stop();
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
rightFront.setBrake(brakeType::coast);
leftFront.setBrake(brakeType::coast);
rightBack.setBrake(brakeType::coast);
leftBack.setBrake(brakeType::coast);

//deploySkills();

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

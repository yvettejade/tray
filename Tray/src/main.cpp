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
  motor_group leftDrive(leftFront, leftBack);
  motor_group rightDrive(rightFront, rightBack);
  smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

//float LI[24]={-22.266667, -39.333333, -47.266667, -69.400000, -44.000000, -56.333333, -51.266667, -45.200000, -37.133333, -53.800000, -42.600000, -49.066667, -51.733333, -55.066667, -39.600000, -57.066667, 14.533333, 0.000000, -2.000000, -5.333333, 3.400000, 4.666667, 0.000000, -3.866667};
//float RI[24]={-21.066667, -38.466667, -43.666667, -70.800000, -40.200000, -46.266667, -59.200000, -41.800000, -31.533333, -32.733333, -43.800000, -53.466667, -55.733333, -63.600000, -27.933333, -51.000000, -4.600000, 3.000000, -0.000000, -2.066667, -2.000000, -2.066667, 1.000000, -2.933333};
float RI[9]={-70.533333, -145.666667, -114.866667, -121.666667, -130.666667, -132.533333, -98.000000, -0.000000, 2.200000};
float LI[9]={-63.866667, -139.866667, -112.066667, -124.066667, -123.266667, -106.066667, -96.800000, 0.000000, -14.333333};
for(int a=0; a<9; a++)
{
  leftIntake.spin(directionType::fwd, LI[a], velocityUnits::rpm);
  rightIntake.spin(directionType::fwd, RI[a], velocityUnits::rpm);
  if(a<8)
    task::sleep(100);
}
rightIntake.stop();
leftIntake.stop();

 tilter.setBrake(brakeType::hold);
 double speedTilter=-0.005*tilter.rotation(rotationUnits::raw)+90;
 bool start=false;
 while(tilter.rotation(rotationUnits::raw)<dropOff)
 {
   if(tilter.rotation(rotationUnits::raw)>2300)
   {
     tilter.setBrake(brakeType::coast);
   }
   if(start==false && tilter.rotation(rotationUnits::raw)>=3800)
    Drivetrain.driveFor(directionType::fwd, (18.5*0.1), distanceUnits::in, 10, velocityUnits::pct,0);
   speedTilter=-0.005*tilter.rotation(rotationUnits::raw)+90;
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
  
}
void deploy()
{
    motor_group leftDrive(leftFront, leftBack);
    motor_group rightDrive(rightFront, rightBack);
    smartdrive Drivetrain= smartdrive(leftDrive, rightDrive, gyroacc, 12.56, 9.25, 8, distanceUnits::in, 1);

    /*Drivetrain.driveFor(directionType::fwd, (18.5*0.3), distanceUnits::in, 40, velocityUnits::pct,1);
    task::sleep(300);
    Drivetrain.driveFor(directionType::rev, (18.5*0.2), distanceUnits::in, 40, velocityUnits::pct,1);
    task::sleep(300);*/
    
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
  rightTurn(1.1);
  task::sleep(300);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 6, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(500);
  rightTurn(0.35);
  rightIntake.stop();
  leftIntake.stop();
  Drivetrain.driveFor(directionType::rev, 18.5*1.3, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(300);
  rightTurn(0.8);
  task::sleep(300);
  rightIntake.spin(directionType::fwd, 100, percentUnits::pct);
  leftIntake.spin(directionType::fwd, 100, percentUnits::pct);
  Drivetrain.driveFor(directionType::fwd, 20, distanceUnits::in, 40, velocityUnits::pct);
  task::sleep(500);
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
 deploy();   
 
 //redLarge();  
 //redSmall();
 blueLarge();
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
 


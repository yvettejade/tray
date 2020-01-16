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
#include "fstream"

using namespace vex;
using namespace std;
competition Competition;
 
int dropOff=4600;
int armLiftHigh=2850;
int armLiftLow=2075;
bool driving=false;
bool trayUp=false;

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
bool deploy=false;
int fullTask()
{
while(true)
{
  if(Controller1.ButtonUp.pressing())
  {
    rightIntake.setBrake(brakeType::hold);
    leftIntake.setBrake(brakeType::hold);
    lift.setBrake(brakeType::hold);
    tilter.setBrake(brakeType::coast);
      Controller1.Screen.print("%s","here");
    lift.startSpinTo(2300, rotationUnits::raw, 100, velocityUnits::pct);
    leftIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    rightIntake.startSpinTo(-20000, rotationUnits::raw, 100, velocityUnits::pct);
    tilter.spinTo(1600, rotationUnits::raw, 100, velocityUnits::pct);
    vex::task::sleep(500);
    leftIntake.stop();
    rightIntake.stop();
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
      deploy=true;
    while(Controller1.ButtonUp.pressing())
    {
      task::sleep(10);
    }
  }
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
    lift.resetRotation();
    tilter.resetRotation();
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
        tilter.setBrake(brakeType::coast);
        
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
int logTask()
{
  Brain.resetTimer();
  while(true)
  {
    if(deploy==true)
    {
      Controller1.Screen.print("%s", "goiiiiiiiiiiiing");
      printf("LF %6.6f\n",leftFront.velocity(velocityUnits::rpm));
      printf("RF %6.6f\n",rightFront.velocity(velocityUnits::rpm));
      printf("LB %6.6f\n",leftBack.velocity(velocityUnits::rpm));
      printf("RB %6.6f\n",rightBack.velocity(velocityUnits::rpm));
      printf("LI %6.6f\n",leftIntake.velocity(velocityUnits::rpm));
      printf("RI %6.6f\n",rightIntake.velocity(velocityUnits::rpm));
      printf("LT %6.6f\n",lift.velocity(velocityUnits::rpm));
      printf("TL %6.6f\n",tilter.velocity(velocityUnits::rpm));
    }
    
    task::sleep(100);
  }
  return 0;
}
void autonomous( void )
{
  
}
void usercontrol( void )
{
rightIntake.setBrake(brakeType::hold);
leftIntake.setBrake(brakeType::hold);
lift.setBrake(brakeType::hold);
tilter.setBrake(brakeType::coast);

vex::task d(driveTask);
vex::task f(fullTask);
vex::task p(printTask);
vex::task l(logTask);
/*float RF[115]={0.000000, 0.000000, 0.000000, 0.000000, 16.066667, 25.000000, 40.800000, 61.266667, 74.933333, 86.200000, 87.000000, 91.133333, 97.000000, 91.000000, 95.333333, 96.000000, 96.800000, 95.733333, 97.400000, 99.400000, 66.866667, 69.466667, 59.200000, 52.733333, 43.866667, 10.000000, -11.733333, -28.400000, -40.400000, -36.600000, -41.800000, -17.333333, -7.466667, -11.133333, -8.066667, 0.000000, 14.800000, 18.800000, 24.400000, 35.533333, 44.266667, 48.066667, 56.133333, 53.266667, 58.400000, 60.400000, 60.000000, 59.533333, 62.600000, 57.000000, 20.933333, 47.466667, 37.133333, 30.266667, 25.600000, 15.466667, 7.200000, 9.000000, 0.000000, -5.266667, -7.800000, -28.333333, -8.000000, -13.200000, -17.533333, -18.733333, -20.866667, -24.066667, -7.533333, -18.866667, -7.733333, -3.800000, -7.133333, -7.800000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 24.133333, 7.533333, 0.000000, 0.000000, 0.000000, -5.333333, -7.066667, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
float RB[115]={0.000000, 0.000000, 0.000000, 0.000000, 16.933333, 25.600000, 41.133333, 61.800000, 74.666667, 86.666667, 85.266667, 93.066667, 95.066667, 92.466667, 93.666667, 94.533333, 94.200000, 92.466667, 96.466667, 97.800000, 69.200000, 68.266667, 59.466667, 50.733333, 43.200000, 10.733333, -8.400000, -27.200000, -38.866667, -36.000000, -40.800000, -19.000000, -6.066667, -10.200000, -8.000000, 0.000000, 13.600000, 18.800000, 23.866667, 35.266667, 42.333333, 48.466667, 54.933333, 52.733333, 59.400000, 58.933333, 58.666667, 59.666667, 60.600000, 55.333333, 22.733333, 51.200000, 35.466667, 26.200000, 23.333333, 14.666667, 6.266667, 7.533333, 0.000000, -3.533333, -7.733333, -28.133333, -7.733333, -13.400000, -17.400000, -17.800000, -21.000000, -23.800000, -8.400000, -12.200000, -6.400000, -12.533333, -3.533333, 5.066667, 5.066667, 0.000000, 0.000000, 0.000000, 4.000000, 0.000000, 0.000000, 0.000000, -2.800000, -2.800000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 25.733333, 5.933333, -4.600000, -70.800000, -5.000000, -5.200000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
float LF[115]={-0.000000, -0.000000, -0.000000, 5.133333, 7.600000, 24.733333, 44.533333, 58.000000, 73.266667, 84.200000, 82.600000, 90.600000, 92.933333, 92.533333, 97.066667, 91.733333, 94.000000, 93.733333, 92.933333, 96.933333, 61.600000, 73.466667, 64.400000, 49.733333, 33.266667, 34.733333, 35.266667, 41.800000, 58.466667, 68.866667, 73.533333, 69.400000, 38.666667, 42.600000, 32.066667, 26.000000, 10.466667, 12.600000, 18.133333, 35.533333, 45.266667, 51.800000, 39.533333, 53.200000, 40.133333, 35.333333, 40.933333, 53.533333, 47.333333, 58.733333, 68.266667, 63.333333, 31.933333, 45.000000, 23.666667, 10.133333, 7.733333, -7.933333, -14.800000, -13.133333, -10.333333, -12.866667, -8.400000, -14.400000, -23.066667, -26.866667, -31.533333, -32.066667, -4.866667, -17.533333, -8.800000, -14.666667, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 2.600000, 2.200000, -0.000000, -0.000000, -0.000000, -2.533333, -0.000000, -2.533333, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 4.666667, 15.133333, 2.533333, -7.066667, -0.000000, -8.866667, -9.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000};
float LB[115]={-0.000000, -0.000000, -0.000000, 11.733333, 11.533333, 23.600000, 44.066667, 60.400000, 74.466667, 84.933333, 89.400000, 92.733333, 91.933333, 90.533333, 92.733333, 97.533333, 99.466667, 94.000000, 95.733333, 96.533333, 64.600000, 74.533333, 64.000000, 49.200000, 30.866667, 35.066667, 35.866667, 42.000000, 59.466667, 74.800000, 74.466667, 71.200000, 35.200000, 43.000000, 31.200000, 23.066667, 10.466667, 12.400000, 19.333333, 35.800000, 45.666667, 51.466667, 42.333333, 61.866667, 38.600000, 31.666667, 42.133333, 53.933333, 47.400000, 61.200000, 68.266667, 63.200000, 36.133333, 53.066667, 20.733333, 7.600000, 5.666667, -5.266667, -15.333333, -12.066667, -10.733333, -14.733333, -9.400000, -13.600000, -23.733333, -28.400000, -32.133333, -33.266667, -4.200000, -19.933333, -6.200000, -5.266667, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 4.666667, -0.000000, -0.000000, -0.000000, -8.933333, -8.933333, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 4.400000, 7.400000, -4.466667, -3.866667, -5.733333, -6.600000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000};
float LI[115]={0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -29.466667, -55.666667, -50.066667, -51.200000, -49.066667, -49.866667, -47.866667, -49.066667, -50.733333, -49.000000, -51.266667, -51.800000, -50.333333, -50.400000, -49.800000, -49.133333, 15.133333, -2.200000, 5.200000, 0.000000, 0.000000, -29.733333, -55.666667, -48.200000, -50.200000, -49.800000, -49.200000, 12.800000, -1.466667, 2.600000, 0.000000, -29.000000, -55.933333, -50.933333, -50.266667, -48.400000, -47.600000, -49.200000, -50.133333, 19.933333, 0.000000, -28.800000, -55.400000, -48.333333, -49.800000, -49.466667, 16.333333, -1.800000, 1.200000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -27.666667, -52.733333, -48.666667, -49.400000, -50.866667, -50.266667, 3.466667, -6.466667, 0.000000, 0.000000, -28.666667, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
float RI[115]={-0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -32.000000, -55.333333, -46.066667, -52.000000, -49.466667, -49.933333, -49.000000, -50.066667, -49.466667, -50.400000, -50.466667, -50.400000, -48.666667, -50.933333, -49.533333, -50.066667, 8.200000, 1.400000, -2.666667, -1.333333, -0.000000, -27.866667, -53.933333, -49.266667, -50.133333, -50.933333, -50.866667, 14.600000, -5.600000, -0.000000, -90.200000, -30.800000, -53.066667, -49.933333, -50.933333, -51.066667, -49.533333, -51.333333, -51.066667, 16.000000, -0.000000, -28.533333, -52.866667, -50.066667, -50.933333, -50.133333, 15.866667, -3.266667, -2.266667, -0.000000, 2.000000, -2.133333, -0.000000, -0.000000, 1.866667, -2.200000, -0.000000, -24.666667, -54.133333, -49.933333, -51.400000, -50.933333, -49.933333, 4.133333, -5.000000, -0.000000, -10.866667, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000};
float LT[115]={-0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 192.000000, 166.133333, 180.266667, 169.533333, 172.133333, 170.466667, 172.066667, 163.600000, 178.866667, 173.400000, 171.000000, -9.933333, -0.000000, -5.666667, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -79.066667, -214.666667, -219.266667, -179.533333, -209.400000, -220.733333, -181.066667, -207.333333, -218.333333, -49.733333, 5.466667, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000};
float TL[115]={0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 192.800000, 177.800000, 195.800000, 184.866667, 187.666667, 139.333333, 37.000000, -10.933333, -10.066667, 0.000000, 0.000000, 0.000000, 3.933333, -5.533333, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -1.533333, 2.866667, 0.000000, 0.000000, -6.066667, -8.733333, -4.266667, 0.000000, -85.466667, -210.866667, -202.533333, -208.333333, -192.866667, -184.266667, 26.466667, 47.933333, 26.800000, 8.466667, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
for(int a=0; a<115; a++)
{
  rightFront.spin(directionType::fwd, RF[a], velocityUnits::rpm);
  rightBack.spin(directionType::fwd, RB[a], velocityUnits::rpm);
  leftFront.spin(directionType::fwd, LF[a], velocityUnits::rpm);
  leftBack.spin(directionType::fwd, LB[a], velocityUnits::rpm);
  leftIntake.spin(directionType::fwd, LI[a], velocityUnits::rpm);
  rightIntake.spin(directionType::fwd, RI[a], velocityUnits::rpm);
  lift.spin(directionType::fwd, LT[a], velocityUnits::rpm);
  tilter.spin(directionType::fwd, TL[a], velocityUnits::rpm);
  task::sleep(100);
}
rightFront.stop();
rightBack.stop();
leftFront.stop();
leftBack.stop();
rightIntake.stop();
leftIntake.stop();
tilter.stop();
lift.stop();*/

Controller1.Screen.print("%s","done");
while (1)
{
  vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
}
}
int main() {
  //Run the pre-autonomous function.
  vexcodeInit();
 
  //Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol( usercontrol );
  Competition.autonomous( autonomous );
 
  //Prevent main from exiting with an infinite loop.                      
  while(1) 
  {
    vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
  }  
   
}
 


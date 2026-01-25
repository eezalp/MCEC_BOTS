/*----------------------------------------------------------------------------*/
/*                                                                            */
/*  Module:       main.cpp                                                    */
/*  Author:      student                                                      */
/*  Created:       10/1/2025, 2:15:24 PM                                      */
/*  Description:    V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------
|         Brain Port Assignments             |
|   Right Motor 1 : PORT 1 | PORT 11: No Worky      |
|   Right Motor 2 : PORT 2 | PORT 12: No Worky      |
|   Right Motor 3 : PORT 3 | PORT 13: Intake Drive     |
|    Left Motor 1 : PORT 4 | PORT 14: No Worky    |
|        No Worky : PORT 5 | PORT 15: Intake Aimer |
|        No Worky : PORT 6 | PORT 16: None      |
|    Left Motor 2 : PORT 7 | PORT 17: None        |
|    Left Motor 3 : PORT 8 | PORT 18: None        |
|   : PORT 9 | PORT 19:            |
|     : PORT 10 | PORT 20:           |
|           PORT 21: Radio                |
-----------------------------------------------------------------*/

/*---------------------
 | Brain 3-Wire Ports |
 |   A: None          |
 |   B: None          |
 |   C: None          |
 |   D: None          |
 |   E: None          |
 |   F: None          |
 |   G: None          |
 |   H: None          |
 ---------------------*/

/*---------------------------------------------------------------- 
|           Controls                   |
| L2: Outtake                            R2: Intake        |
| L1: None                                  R1: None        |
|                                                                |
| Ls3: F/B                                  Rs2: L/R        |
| Ls4: None                                Rs1: None         |
|          Up: None                  X:None              |
| Left: None    Right: None    Y:None      A:Sort Wheel Go      |
|          Down: None                  B:None              |
----------------------------------------------------------------*/

#include "MCEC_Objects.h"
#include <cmath>

#define TURN_ONLY_MULTI 0.4f
#define DRIVE_MULTI  0.75f

vex::brain Brain;
vex::controller controller = vex::controller();
vex::competition comp = vex::competition();
vex::inertial inertial = vex::inertial(vex::PORT6);

vex::motor crabLeft  = vex::motor(vex::PORT20);
vex::motor crabRight = vex::motor(vex::PORT10);

vex::motor a = vex::motor(vex::PORT1);
vex::motor b = vex::motor(vex::PORT2);
vex::motor c = vex::motor(vex::PORT3);

vex::motor d = vex::motor(vex::PORT4);
vex::motor e = vex::motor(vex::PORT7);
vex::motor f = vex::motor(vex::PORT8);

MCEC::Drivetrain6 drivetrain(
  vex::PORT1, vex::PORT2, vex::PORT3, 
  vex::PORT4, vex::PORT7, vex::PORT8
);

MCEC::Controller controls;

void IntakeGo(){
  crabLeft.spin(vex::forward, 200, vex::rpm);
  crabRight.spin(vex::reverse, 200, vex::rpm);
}

void IntakeNotGo(){
  crabLeft.spin(vex::reverse, 200, vex::rpm);
  crabRight.spin(vex::forward, 200, vex::rpm);
}

void IntakeStop(){
  crabLeft.stop();
  crabRight.stop();
}

void DriverLoop(){
  controls.Set();

  if(controls.lStick.isMoved() || controls.rStick.isMoved()){
    if(controls.lStick.y < 10){
      drivetrain.Drive(0, controls.rStick.x * TURN_ONLY_MULTI);
    }else{
      drivetrain.Drive(controls.lStick.y * DRIVE_MULTI, controls.rStick.x * DRIVE_MULTI);
    }
    // drivetrain.rightMotors.spin(vex::forward);
    // drivetrain.leftMotors.spin(vex::forward);
  }else{
    drivetrain.Stop(vex::brakeType::brake);
  }
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%f", drivetrain.curPowerL);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%f", drivetrain.curPowerR);
}

void Drive(){
  while (comp.isDriverControl()){
    DriverLoop();
    vex::this_thread::sleep_for(10);
  }
}

void Auton(){
  return;
  // Win
  drivetrain.Spin((48 * (3.0f / 5.0f)) / wheelCirc);

  vex::this_thread::sleep_for(600);

  drivetrain.Rotate(-90);

  vex::this_thread::sleep_for(600);

  drivetrain.Spin((4 * (3.0f / 5.0f)) / wheelCirc);

  vex::this_thread::sleep_for(400);

  // IntakeGo();

  vex::this_thread::sleep_for(3000);

  IntakeStop();

  vex::this_thread::sleep_for(400);

  drivetrain.Spin((-24 * (3.0f / 5.0f)) / wheelCirc);

  IntakeGo();

  vex::this_thread::sleep_for(3000);

  IntakeStop();
}

void SetControls(){
  controls.R2.SetOnPress(IntakeGo);
  controls.L2.SetOnPress(IntakeNotGo);

  controls.R2.SetOnRelease(IntakeStop);
  controls.L2.SetOnRelease(IntakeStop);

  
  if(!comp.isFieldControl()){
      controls.Down.SetOnPress(Auton);
  }
}

int main(){
  controller.rumble(". . .");

  vex::this_thread::sleep_for(1000);
  Brain.Screen.print("runmble ");

  if(comp.isFieldControl()){
    comp.drivercontrol(Drive);
    comp.autonomous(Auton);
  }

  SetControls();

  // drivetrain.rightMotors = vex::motor_group(d, e, f);
  // drivetrain.leftMotors = vex::motor_group(a, b, c);
  // drivetrain.Init();

  
  drivetrain.SetInertial(&inertial);
  drivetrain.ResetPositions();
  drivetrain.wheelRadius = 3.25f / 2;
  drivetrain.wheelDistance = 12.0f;

  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
    vex::this_thread::sleep_for(10);
  }
}

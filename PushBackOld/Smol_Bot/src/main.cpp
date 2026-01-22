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

vex::brain Brain;
vex::controller controller = vex::controller();
vex::competition comp = vex::competition();
vex::inertial inertial = vex::inertial(vex::PORT20);

vex::motor crabLeft  = vex::motor(vex::PORT20);
vex::motor crabRight = vex::motor(vex::PORT10);

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
    drivetrain.Drive(controls.lStick.y, controls.rStick.x);
  }else{
    drivetrain.Stop(vex::brakeType::coast);
  }
}

void Drive(){
  while (comp.isDriverControl()){
    DriverLoop();
    vex::this_thread::sleep_for(10);
  }
}

void Auton(){
  // Win
}

void SetControls(){
  controls.R2.SetOnPress(IntakeGo);
  controls.L2.SetOnPress(IntakeNotGo);
  
  controls.R2.SetOnRelease(IntakeStop);
  controls.L2.SetOnRelease(IntakeStop);
}

int main(){
  inertial.calibrate(3);

  inertial.resetHeading();
  while(inertial.isCalibrating());
  inertial.setHeading(0, vex::deg);
  inertial.setRotation(0, vex::deg);
  controller.rumble(". . .");

  SetControls();

  comp.drivercontrol(Drive);
  comp.autonomous(Auton);

  drivetrain.Reset();

  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
    vex::this_thread::sleep_for(10);
  }
}

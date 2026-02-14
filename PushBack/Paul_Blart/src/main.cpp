/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/1/2025, 2:15:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------
|                 Brain Port Assignments                         |
|     Right Motor 1: PORT 1 | PORT 11: Intake Mid Motor          |
|     Right Motor 2: PORT 2 | PORT 12: Intake Top Motor          |
|     Right Motor 3: PORT 3 | PORT 13: Sorter Motor Driver       |
|     Right Motor 4: PORT 4 | PORT 14: Sorter Door Driver        |
|      Left Motor 5: PORT 5 | PORT 15: Turret Driver             |
|      Left Motor 6: PORT 6 | PORT 16: Turret Rollers            |
|      Left Motor 7: PORT 7 | PORT 17: Shiv Motor                |
|      Left Motor 8: PORT 8 | PORT 18: Color Sensor              |
|          Inertial: PORT 9 | PORT 19: None                      |
|       Intake Top: PORT 10 | PORT 20: None                      |
|                     PORT 21: Radio                             |
-----------------------------------------------------------------*/

/*-------------------
 | Brain 3-Wire Ports |
 |   A: Turret Pot    |
 |   B: Shiv Uppies   |
 |   C: Turret Up     |
 |   D: Front Gate    |
 |   E: None          |
 |   F: None          |
 |   G: None          |
 |   H: None          |
 ---------------------*/



/*---------------------------------------------------------------- 
|                     Controls                                   |
| L2: Outtake                            R2: Intake              |
| L1: None                               R1: None                |
|                                                                |
| Ls3: F/B                               Rs2: L/R                |
| Ls4: None                              Rs1: None               |
|       Up: None                 X: Turret                       |
| Left: None  Right: None    Y:None   A: Sort Wheel Go           |
|       Down: None               B:None                          |
----------------------------------------------------------------*/

#include <MCEC_Objects.h>

#define TURRET_MAX_ANGLE 23
#define TURRET_MIN_ANGLE 11


vex::competition comp;
vex::brain Brain;

MCEC::SwerveDrive drivetrain(
  vex::PORT4 , vex::PORT5 , vex::PORT6 , // front left 
  vex::PORT1 , vex::PORT2 , vex::PORT3 , // front right
  vex::PORT7 , vex::PORT8 , vex::PORT9 , // back left
  vex::PORT10, vex::PORT11, vex::PORT12  // back left
);

// Motors
  vex::motor intakeb = vex::motor(vex::PORT18);
  vex::motor intakem = vex::motor(vex::PORT19);
  vex::motor shoot   = vex::motor(vex::PORT20);

  vex::inertial inertial = vex::inertial(vex::PORT15);


// vex::controller
MCEC::Controller controls = MCEC::Controller();


void StopMoving(){
  drivetrain.Stop();
}

void DriverLoop(){
  controls.Set();
  if(controls.lStick.isMoved() || controls.rStick.isMoved()){
    float x = controls.rStick.x / 100.0f;
    drivetrain.Drive(Vector2(controls.lStick.x, controls.lStick.y), x);
  }else{
    drivetrain.Stop(vex::brakeType::coast);
  }

}

void IntakeStore(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
}
void IntakeNotStore(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
}
void IntakeGo(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
  shoot.spin(vex::reverse, 80, vex::pct);
}
void IntakeNotGo(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::forward, 80, vex::pct);
}
void IntakeStop(){
  intakeb.stop();
  intakem.stop();
  shoot.stop();
}

void ShivDown(){

}
void ShivUp(){
  
}

void TurretUp(){

}
void TurretDown(){
  
}

void Driver(){
  controls.controller.rumble("..");

  while(comp.isDriverControl()){
    DriverLoop();
  }
}

void Auton(){

}

void SetControls(){
  controls.R1.SetOnPress(IntakeGo);
  controls.L1.SetOnPress(IntakeNotGo);
  controls.R2.SetOnPress(IntakeStore);
  controls.L2.SetOnPress(IntakeNotStore);

  controls.R1.SetOnRelease(IntakeStop);
  controls.L1.SetOnRelease(IntakeStop);
  controls.R2.SetOnRelease(IntakeStop);
  controls.L2.SetOnRelease(IntakeStop);
}

void InitInertial(){
  inertial.calibrate(3);

  inertial.resetHeading();
  while(inertial.isCalibrating());
  inertial.setHeading(0, vex::deg);
  inertial.setRotation(0, vex::deg);
}

void SetupFieldControl(){
  if(comp.isFieldControl()){
    controls.controller.Screen.clearScreen();
    controls.controller.Screen.print("Field Control");
    comp.drivercontrol(Driver);
    comp.autonomous(Auton);
  }
}

int main(){
  InitInertial();
  SetControls();
  SetupFieldControl();

  drivetrain.SetRotationOffsets(
    129.28f, //fl
    78.04f, //fr
    129.28f, //bl
    345.49f //br
  );
  drivetrain.frontLeft.SetPIDVariables(0.1f, 0.001f, 0.01f);
  drivetrain.frontRight.SetPIDVariables(0.1f, 0.001f, 0.01f);
  drivetrain.backLeft.SetPIDVariables(0.1f, 0.001f, 0.01f);
  drivetrain.backRight.SetPIDVariables(0.1f, 0.001f, 0.01f);

  controls.controller.rumble(".");
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
    vex::this_thread::sleep_for(10);
  }
}

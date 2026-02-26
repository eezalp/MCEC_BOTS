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
  vex::PORT13, vex::PORT12, vex::PORT11, // front left // former front right
  vex::PORT18, vex::PORT19, vex::PORT20, // front right // former back right
  vex::PORT3 , vex::PORT2 , vex::PORT1 ,// back left // former back left
  vex::PORT8 , vex::PORT9 , vex::PORT10 // back right // former front left
);

// Motors
  vex::motor intakeb = vex::motor(vex::PORT5);
  vex::motor intakem = vex::motor(vex::PORT6);
  vex::motor shoot   = vex::motor(vex::PORT7);

  vex::inertial inertial = vex::inertial(vex::PORT16);


// vex::controller
MCEC::Controller controls = MCEC::Controller();

vex::digital_out turret(Brain.ThreeWirePort.B);


void StopMoving(){
  drivetrain.Stop(vex::brakeType::hold);
}

void DriverLoop(){
  static Vector2 lastJoystick = Vector2(0, 0);
  controls.Set();
  if(controls.lStick.isMoved() || controls.rStick.isMoved()){
    Vector2 controllerVector(-controls.lStick.y, -controls.lStick.x);
    float angleDiff = MCEC::abs2(MCEC::AngleDiff(controllerVector.GetAngle(), lastJoystick.GetAngle()));
    float x = controls.rStick.x;
    
    // if(angleDiff > 2.0f){
    //   drivetrain.Drive(controllerVector, x);
    //   controls.controller.Screen.setCursor(1, 1);
    //   controls.controller.Screen.print("%.4fc  ", controllerVector.GetAngle());
    //   controls.controller.Screen.setCursor(2, 1);
    //   controls.controller.Screen.print("%.4fc  ", controllerVector.GetMagnitude());
    //   lastJoystick = controllerVector;
    // }else{
    //   drivetrain.Drive(lastJoystick.Normalize() * controllerVector.GetMagnitude(), x);
    //   controls.controller.Screen.setCursor(1, 1);
    //   controls.controller.Screen.print("%.4fl  ", lastJoystick.GetAngle());
    //   controls.controller.Screen.setCursor(2, 1);
    //   controls.controller.Screen.print("%.4fl  ", lastJoystick.GetMagnitude());
    // }
      drivetrain.Drive(controllerVector, x);

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
  shoot.spin(vex::forward, 80, vex::pct);
}
void IntakeNotGo(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::reverse, 80, vex::pct);
}
void IntakeStop(){
  intakeb.stop();
  intakem.stop();
  shoot.stop();
}

void ExitBlartMode();

void EnterBlartMode(){
  drivetrain.EnterCosplineMode();
  controls.controller.Screen.setCursor(3, 1);
  controls.controller.Screen.print("Blart Mode Activated");
  controls.Y.SetOnPress(ExitBlartMode);
}

void ExitBlartMode(){
  drivetrain.ExitCosplineMode();
  controls.controller.Screen.clearScreen();
  controls.Y.SetOnPress(EnterBlartMode);
}

void ShivDown(){

}
void ShivUp(){
  
}

void TurretDown();
void TurretUp(){
  turret.set(true);
  controls.A.SetOnPress(TurretDown);
}
void TurretDown(){
  turret.set(false);
  controls.A.SetOnPress(TurretUp);
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

  controls.A.SetOnPress(TurretDown);
  controls.Y.SetOnPress(ExitBlartMode);

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

//what does PID stand for????
//PROPORTIONAL, INTEGRAL, DERIVATIVE
float PID[] = {0.5f, 0.05f, 0.02f};

int main(){
  InitInertial();
  SetControls();
  SetupFieldControl();

  drivetrain.SetRotationOffsets(
    //updated
    1.14f, //fl 3
    173.40f, //fr 9
    272.90f - 180, //bl 6
    41.48f + 270 //br 12
  );
  drivetrain.frontLeft.SetPIDVariables (0.4f, 0.0f, 0.008f);
  drivetrain.frontRight.SetPIDVariables(0.4f, 0.0f, 0.008f);
  drivetrain.backLeft.SetPIDVariables  (0.4f, 0.0f, 0.008f);
  drivetrain.backRight.SetPIDVariables (0.4f, 0.0f, 0.008f);
  // drivetrain.frontRight.SetPIDVariables(0.0f, 0.0f, 0.00f);
  // drivetrain.backLeft.SetPIDVariables  (0.0f, 0.0f, 0.00f);
  // drivetrain.backRight.SetPIDVariables (0.0f, 0.0f, 0.00f);

  controls.controller.rumble(".");
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
    vex::this_thread::sleep_for(10);
  }
}

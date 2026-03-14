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

#include "MCEC_Objects.h"

#define TURRET_MAX_ANGLE 23
#define TURRET_MIN_ANGLE 11

vex::mutex MCEC::screenMutex;
vex::competition comp;
vex::brain Brain;

MCEC::SwerveDrive drivetrain(
  vex::PORT13, vex::PORT12, vex::PORT11, // front left // former front right
  vex::PORT18, vex::PORT19, vex::PORT20, // front right // former back right
  vex::PORT3 , vex::PORT2 , vex::PORT1 , // back left // former back left
  vex::PORT8 , vex::PORT9 , vex::PORT10, // back right // former front left
  Vector2(7.25f, -1.375f), vex::PORT21, // forward encoder
  Vector2(-4.5f, -0.125f), vex::PORT4, // lateral encoder // port 15 when driver and 4 when auton
  2.0f  // odom wheel radius
);

// Motors
  vex::motor intakeb = vex::motor(vex::PORT5, true);
  vex::motor intakem = vex::motor(vex::PORT6, true);
  vex::motor shoot   = vex::motor(vex::PORT7, true);

  vex::inertial inertial = vex::inertial(vex::PORT17);

  vex::motor descoreMotor = vex::motor(vex::PORT4, true);

// vex::controller
MCEC::Controller controls = MCEC::Controller();

vex::digital_out turret(Brain.ThreeWirePort.A);
vex::digital_out shiv(Brain.ThreeWirePort.C);

bool turretUp = false;
bool shivUp = false;

enum DescorePositions{Stored, Targeting, HighGoal};
DescorePositions descore = DescorePositions::Stored;



void StopMoving(){
  drivetrain.Stop(vex::brakeType::hold);
}

void PosUpdate(){
  static float start = (vex::timer::system()), end;
  static float lastTime;
  float curTime = (vex::timer::system());
  float dt;
  dt = curTime - lastTime;
  lastTime = curTime;

  while(true){
    if(!drivetrain.runningAuton)
      drivetrain.UpdatePosition(dt);

    vex::this_thread::sleep_for(20);
  }
}

void DriverLoop(){
  static Vector2 lastJoystick = Vector2(0, 0);
  controls.Set();
  if(drivetrain.runningAuton) return;

  if(controls.lStick.isMoved() || controls.rStick.isMoved()){
    Vector2 controllerVector(-controls.lStick.y, -controls.lStick.x);
    float angleDiff = MCEC::abs2(MCEC::AngleDiff(controllerVector.GetAngle(), lastJoystick.GetAngle()));
    float x = -controls.rStick.x;
    
    drivetrain.Drive(controllerVector, x);

  }else{
    drivetrain.Stop(vex::brakeType::coast);
  }
  
  // controls.controller.Screen.setCursor(3, 1);
  // controls.controller.Screen.print("%.2f, %.2f", drivetrain.currentPos.x, drivetrain.currentPos.y);
  vex::this_thread::sleep_for(20);
}

void IntakeStore(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
  shoot.spin(vex::reverse, 25, vex::pct);
}
void IntakeNotStore(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::reverse, 25, vex::pct);
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

void ShivUp();

void ShivDown(){
  if(turretUp) return;
  controls.A.SetOnPress(ShivUp);
  shivUp = false;
  shiv = true;
}
void ShivUp(){
  controls.A.SetOnPress(ShivDown);
  shivUp = true;
  shiv = false;
}

void TurretDown();

void TurretUp(){
  if(!shivUp) return;
  turret.set(true);
  turretUp = true;
}
void TurretDown(){
  turret.set(false);
  turretUp = false;
}

void Driver(){
  controls.controller.rumble("..");
  controls.controller.Screen.clearScreen();
  vex::thread Thread = vex::thread(PosUpdate);

  while(1){
    DriverLoop();
    // wait(20, vex::msec);
  }
}

void Auton(){
  controls.controller.Screen.clearScreen();
  drivetrain.points = {
    {{0, 0}, 90, 90},
    {{0, 0}, 90, 90},
    {{0, 0}, 90, 90}
  };
  drivetrain.GoToPoint();
}

void ResetPosition(){
  drivetrain.currentPos = Vector2::zero;
}

void FaceDriver(){
  drivetrain.FaceDirection(180);
}

void FaceOpponent(){
  drivetrain.FaceDirection(0);
}

void FaceLeft(){
  drivetrain.FaceDirection(90);
}

void FaceRight(){
  drivetrain.FaceDirection(270);
}


#define STORED_POS 3.0f
#define HIGOAL_POS 2.2f
#define TARGET_POS 2.0f
void StepDescoreForward(){
  controls.controller.Screen.setCursor(2, 1);
  controls.controller.Screen.print("Forward");
  switch(descore){
    case DescorePositions::Stored:
      descore = DescorePositions::Targeting;
      controls.controller.Screen.setCursor(1, 1);
      controls.controller.Screen.print("Target  ");
      descoreMotor.spinToPosition(TARGET_POS, vex::rotationUnits::rev, true);
      break;
    case DescorePositions::Targeting:
      descore = DescorePositions::HighGoal;
      controls.controller.Screen.setCursor(1, 1);
      controls.controller.Screen.print("HighGoal");
      descoreMotor.spinToPosition(HIGOAL_POS, vex::rotationUnits::rev, true);
      break;
    case DescorePositions::HighGoal:
      descore = DescorePositions::Stored;
      controls.controller.Screen.setCursor(1, 1);
      controls.controller.Screen.print("Stored  ");
      descoreMotor.spinToPosition(STORED_POS, vex::rotationUnits::rev, true);
      break;
  }
}

void StepDescoreBackward(){
  controls.controller.Screen.setCursor(2, 1);
  controls.controller.Screen.print("Reverse");
  switch(descore){
    case DescorePositions::Stored:
      descore = DescorePositions::HighGoal;
      descoreMotor.spinToPosition(HIGOAL_POS, vex::rotationUnits::rev, true);
      break;
    case DescorePositions::Targeting:
      descore = DescorePositions::Stored;
      descoreMotor.spinToPosition(STORED_POS, vex::rotationUnits::rev, true);
      break;
    case DescorePositions::HighGoal:
      descore = DescorePositions::Targeting;
      descoreMotor.spinToPosition(TARGET_POS, vex::rotationUnits::rev, true);
      break;
  }
}

void DisableForward(){
  drivetrain.canForward = false;
}

void EnableForward(){
  drivetrain.canForward = true;
}

void SetControls(){
  controls.L2.SetOnPress(IntakeGo);
  controls.L1.SetOnPress(IntakeStore);
  controls.R1.SetOnPress(IntakeNotStore);
  controls.R2.SetOnPress(IntakeNotGo);

  controls.Down.SetOnPress(TurretDown);
  controls.Up.SetOnPress(TurretUp);

  controls.A.SetOnPress(ShivDown);
  controls.X.SetOnPress(ResetPosition);
  controls.B.SetOnPress(DisableForward);
  controls.Y.SetOnPress(Auton);

  controls.B.SetOnRelease(EnableForward);


  // controls.Down.SetOnPress(FaceDriver);
  // controls.Up.SetOnPress(FaceOpponent);
  controls.Left.SetOnPress(StepDescoreBackward);

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
  comp.drivercontrol(Driver);
  if(comp.isFieldControl()){
    controls.controller.Screen.clearScreen();
    controls.controller.Screen.print("Field Control");
    comp.autonomous(Auton);
  }
}

void SetAutonPath(){
}

//what does PID stand for????
//PROPORTIONAL, INTEGRAL, DERIVATIVE
float PID[] = {0.5f, 0.05f, 0.02f};

int main(){
  descoreMotor.setPosition(3, vex::rotationUnits::rev);
  SetAutonPath();
  
  InitInertial();
  SetControls();
  // drivetrain.frontLeft.SetPIDVariables (1.2f, 0.0f, 0.1f);
  // drivetrain.frontRight.SetPIDVariables(1.2f, 0.0f, 0.1f);
  // drivetrain.backLeft.SetPIDVariables  (1.2f, 0.0f, 0.1f);
  // drivetrain.backRight.SetPIDVariables (1.2f, 0.0f, 0.1f);
  drivetrain.frontLeft.SetPIDVariables (0.4f, 0.0f, 0.08f);
  drivetrain.frontRight.SetPIDVariables(0.4f, 0.0f, 0.08f);
  drivetrain.backLeft.SetPIDVariables  (0.4f, 0.0f, 0.08f);
  drivetrain.backRight.SetPIDVariables (0.4f, 0.0f, 0.08f);

  drivetrain.SetAutonPIDVariables(
    0.1f, 0.0f, 0.0f, // movement
    0.1f, 0.0f, 0.02f  // rotation
  );



  // drivetrain.frontRight.SetPIDVariables(0.0f, 0.0f, 0.00f);
  // drivetrain.backLeft.SetPIDVariables  (0.0f, 0.0f, 0.00f);
  // drivetrain.backRight.SetPIDVariables (0.0f, 0.0f, 0.00f);
  SetupFieldControl();

  // controls.controller.Screen.clearScreen();
  // controls.controller.Screen.setCursor(1, 1);
  // controls.controller.Screen.print("%.2f  %.2f", drivetrain.frontLeft.rotation.angle(), drivetrain.frontRight.rotation.angle());
  // controls.controller.Screen.setCursor(3, 1);
  // controls.controller.Screen.print("%.2f  %.2f", drivetrain.backLeft.rotation.angle(), drivetrain.backRight.rotation.angle());


  drivetrain.SetRotationOffsets(
    //updated
    89.03f, //fl 3
    (173.67f), //fr 9
    93.42f, //bl 6
    54.49f + 270 //br 12
  );

  controls.controller.rumble(".....");
  
  // controls.controller.Screen.clearScreen();
  // controls.controller.Screen.setCursor(1, 1);
  // controls.controller.Screen.print("%.2f  %.2f", drivetrain.frontLeft.rotation.angle(), drivetrain.frontRight.rotation.angle());
  // controls.controller.Screen.setCursor(2, 1);
  // controls.controller.Screen.print("%.2f  %.2f", drivetrain.backLeft.rotation.angle(), drivetrain.backRight.rotation.angle());
  while(1) {
    if(!comp.isFieldControl()){
      // DriverLoop();
    }
  }
}

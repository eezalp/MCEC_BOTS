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
-----------------------------------------------------------------  WRONG PORTS*/

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

MCEC::DiffSwervePodSettings  //top, bottom, rotation
  FL(
    vex::PORT2, vex::forward,
    vex::PORT3, vex::reverse, 
    vex::PORT1
  ),
  FR(
    vex::PORT12, vex::forward,
    vex::PORT13, vex::reverse,
    vex::PORT11
  ),
  BL(
    vex::PORT8, vex::forward,
    vex::PORT9, vex::reverse,
    vex::PORT10
  ), 
  BR(
    vex::PORT18, vex::forward,
    vex::PORT19, vex::reverse,
    vex::PORT20
  );

MCEC::DiffSwerveDrive drivetrain(
  FL, FR, BL, BR,
  Vector2(7.25f, -1.375f), vex::PORT14, // forward encoder
  Vector2(-4.5f, -0.125f), vex::PORT15, // lateral encoder // port 15 when driver and 4 when auton
  2.0f  // odom wheel radius
);

#define LEVER_STARTPOS 300
#define LEVER_POSHIGH  270

// Motors
  vex::motor intake = vex::motor(vex::PORT6, true);

  vex::motor matchloadLeft  = vex::motor(vex::PORT5, true);
  vex::motor matchloadRight = vex::motor(vex::PORT4, true);

  vex::motor lever1 = vex::motor(vex::PORT17, true);
  vex::motor lever2 = vex::motor(vex::PORT16, true);
//

vex::inertial inertial = vex::inertial(vex::PORT7);

MCEC::Controller controls = MCEC::Controller(6);

vex::digital_out descore(Brain.ThreeWirePort.B);
vex::digital_out matchloader(Brain.ThreeWirePort.A);
vex::digital_out turret(Brain.ThreeWirePort.C);

bool turretUp = false;
bool matchloadEngaged = false;
bool matchloading = false;
bool leverDown = false;
bool descoreEngaged = false;

enum DescorePositions{Stored, Targeting, HighGoal};
DescorePositions descorePositions = DescorePositions::Stored;


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
      controls.controller.Screen.setCursor(3, 1);
      controls.controller.Screen.print("F:%f, L:%f", drivetrain.forward.GetAbsDistance()/(2 * M_PI), drivetrain.lateral.GetAbsDistance()/(2 * M_PI));

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
  
  vex::this_thread::sleep_for(20);
}

void IntakeGo(){
  if(leverDown) return;
  intake.spin(vex::reverse, 80, vex::pct);
  matchloadLeft.spin(vex::reverse, 80, vex::pct);
  matchloadRight.spin(vex::forward, 80, vex::pct);
}
void IntakeGoSlow(){
  if(leverDown) return;
  intake.spin(vex::forward, 30, vex::pct);
  matchloadLeft.spin(vex::reverse, 30, vex::pct);
  matchloadRight.spin(vex::forward, 30, vex::pct);
}
void IntakeNotGo(){
  intake.spin(vex::forward, 80, vex::pct);
  matchloadLeft.spin(vex::forward, 80, vex::pct);
  matchloadRight.spin(vex::reverse, 80, vex::pct);
}
void IntakeStop(){
  intake.stop();
  matchloadLeft.stop();
  matchloadRight.stop();
}

void Matchload(){
  matchloadLeft.spin(vex::reverse, 100, vex::pct);
  matchloadRight.spin(vex::forward, 100, vex::pct);
}
void StopMatchload(){
  matchloadLeft.stop();
  matchloadRight.stop();
}
bool scoreRunning = false;

void LeverReset(){
  IntakeStop();
  lever2.spinToPosition(0, vex::degrees, false);
  lever1.spinToPosition(300, vex::degrees, true);
  scoreRunning = false;
}

void ScoreAction(){
  int pos = turretUp ? LEVER_POSHIGH : LEVER_STARTPOS;
  IntakeGo();
  leverDown = false;
  scoreRunning = true;
  // go
  if(!turretUp){
    // lever1.setVelocity(50, vex::percent);
    // lever2.setVelocity(50, vex::percent);

    lever1.spinToPosition(0, vex::degrees, true);
    lever2.spinToPosition(300, vex::degrees, false);
  }else{
    // lever1.setVelocity(100, vex::percent);
    // lever2.setVelocity(100, vex::percent);

    lever1.spinToPosition(30, vex::degrees, true);
    lever2.spinToPosition(270, vex::degrees, false);
  }
  // return
  LeverReset();
}
vex::thread scoreThread;
void Score(){
  if(!scoreRunning){
    scoreThread = vex::thread(ScoreAction);
  }else{
    scoreThread.interrupt();
    scoreThread = vex::thread(LeverReset);
  }
}

void TurretDown();

void TurretUp(){
  turret.set(false);
  turretUp = true;
}
void TurretDown(){
  turret.set(true);
  turretUp = false;
}

void DescoreDisengage(){
  descore.set(true);
  descoreEngaged = true;
}
void DescoreEngage(){
  descore.set(false);
  descoreEngaged = false;
}

void MatchloadEngage();

void MatchloadDisengage(){
  matchloader.set(true);
  matchloadEngaged = false;
}
void MatchloadEngage(){
  matchloader.set(false);
  matchloadEngaged = true;
}

void MatchloadToggle(){
  if(matchloadEngaged){
    MatchloadDisengage();
  }else{
    MatchloadEngage();
  }
}

void Driver(){
  controls.controller.rumble(".");
  controls.controller.Screen.clearScreen();
  vex::thread Thread = vex::thread(PosUpdate);

  TurretDown();
  while(1){
    DriverLoop();
    wait(20, vex::msec);
  }
}

void Auton(){
  return;
  // controls.controller.Screen.clearScreen();
  // drivetrain.points = {
  //   {{0, 0}, 90, 90},
  //   {{0, 0}, 90, 90},
  //   {{0, 0}, 90, 90}
  // };
  // drivetrain.GoToPoint();
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
#define HIGOAL_POS 2.0f
#define TARGET_POS 1.6f
void StepDescoreForward(){
  switch(descorePositions){
    case DescorePositions::Stored:
      descore = DescorePositions::Targeting;
      // descoreMotor.spinToPosition(TARGET_POS, vex::rotationUnits::rev, false);
      break;
    case DescorePositions::Targeting:
      descore = DescorePositions::HighGoal;
      // descoreMotor.spinToPosition(HIGOAL_POS, vex::rotationUnits::rev, false);
      break;
    case DescorePositions::HighGoal:
      descore = DescorePositions::Stored;
      // descoreMotor.spinToPosition(STORED_POS, vex::rotationUnits::rev, false);
      break;
  }
}

void StepDescoreBackward(){
  switch(descorePositions){
    case DescorePositions::Stored:
      descore = DescorePositions::HighGoal;
      // descoreMotor.spinToPosition(HIGOAL_POS, vex::rotationUnits::rev, false);
      break;
    case DescorePositions::Targeting:
      descore = DescorePositions::Stored;
      // descoreMotor.spinToPosition(STORED_POS, vex::rotationUnits::rev, false);
      break;
    case DescorePositions::HighGoal:
      descore = DescorePositions::Targeting;
      // descoreMotor.spinToPosition(TARGET_POS, vex::rotationUnits::rev, false);
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
  controls.R2.SetOnPress(IntakeGo);
  controls.L2.SetOnPress(IntakeNotGo);

  controls.Down.SetOnPress(TurretDown);
  controls.Up.SetOnPress(TurretUp);

  controls.A.SetOnPress(Score);
  controls.X.SetOnPress(MatchloadToggle);
  controls.B.SetOnPress(DisableForward);
  controls.Y.SetOnPress(Auton);

  controls.B.SetOnRelease(EnableForward);

  controls.Right.SetOnPress(DescoreDisengage);
  controls.Left.SetOnPress(DescoreEngage);

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
  comp.autonomous(Auton);
  if(comp.isFieldControl()){
    // controls.controller.Screen.clearScreen();
    // controls.controller.Screen.print("Field Control");
  }
}

//what does PID stand for????
//PROPORTIONAL, INTEGRAL, DERIVATIVE
float PID[] = {0.5f, 0.05f, 0.02f};

int main(){
  // descoreMotor.setPosition(3, vex::rotationUnits::rev);
  
  InitInertial();
  SetControls();

  lever1.setVelocity(100, vex::percent);
  lever1.setPosition(LEVER_STARTPOS, vex::deg);
  
  lever2.setVelocity(100, vex::percent);
  lever2.setPosition(0, vex::deg);

  // drivetrain.frontLeft.SetPIDVariables (0.0f, 0.0f, 0.0f);
  // drivetrain.frontRight.SetPIDVariables(0.0f, 0.0f, 0.0f);
  // drivetrain.backLeft.SetPIDVariables  (0.0f, 0.0f, 0.0f);
  // drivetrain.backRight.SetPIDVariables (0.0f, 0.0f, 0.0f);

  drivetrain.frontLeft.SetPIDVariables (0.25f, 0.0f, 0.08f);
  drivetrain.frontRight.SetPIDVariables(0.25f, 0.0f, 0.08f);
  drivetrain.backLeft.SetPIDVariables  (0.25f, 0.0f, 0.08f);
  drivetrain.backRight.SetPIDVariables (0.25f, 0.0f, 0.08f);

  drivetrain.SetAutonPIDVariables(
    0.1f, 0.0f, 0.0f, // movement
    0.1f, 0.0f, 0.02f  // rotation
  );

  SetupFieldControl();

  drivetrain.SetRotationOffsets(
    //updated
    179.47f + 180, //fl
    171.47f + 180, //fr
    275.48f - 180, //bl
    226.58f - 90  //br
  );

  //controls.controller.rumble(".....");

  
  controls.controller.Screen.clearScreen();
  controls.controller.Screen.setCursor(1, 1);
  controls.controller.Screen.print("%.2f  %.2f", drivetrain.frontLeft.rotation.angle(), drivetrain.frontRight.rotation.angle());
  controls.controller.Screen.setCursor(2, 1);
  controls.controller.Screen.print("%.2f  %.2f", drivetrain.backLeft.rotation.angle(), drivetrain.backRight.rotation.angle());
  // controls.controller.Screen.setCursor(3, 1);
  // controls.controller.Screen.print("%d, %d", vex::forward, vex::reverse);
  while(1) {
    if(!comp.isFieldControl()){
      // DriverLoop();
    }
    wait(20, vex::msec);
  }
}
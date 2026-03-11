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


vex::competition comp;
vex::brain Brain;

MCEC::SwerveDrive drivetrain(
  vex::PORT13, vex::PORT12, vex::PORT11, // front left // former front right
  vex::PORT18, vex::PORT19, vex::PORT20, // front right // former back right
  vex::PORT3 , vex::PORT2 , vex::PORT1 , // back left // former back left
  vex::PORT8 , vex::PORT9 , vex::PORT10, // back right // former front left
  Vector2(7.25f, -1.375f), vex::PORT21, // forward encoder
  Vector2(-4.5f, -0.125f), vex::PORT4, // lateral encoder
  1.25f  // odom wheel radius
);

// Motors
  vex::motor intakef = vex::motor(vex::PORT21, true);
  vex::motor intakeb = vex::motor(vex::PORT5, true);
  vex::motor intakem = vex::motor(vex::PORT6, true);
  vex::motor shoot   = vex::motor(vex::PORT7, true);

  vex::inertial inertial = vex::inertial(vex::PORT17);


// vex::controller
MCEC::Controller controls = MCEC::Controller();

vex::digital_out turret(Brain.ThreeWirePort.A);
vex::digital_out shiv(Brain.ThreeWirePort.C);

float dt;
bool turretUp = false;
bool shivUp = false;

void StopMoving(){
  drivetrain.Stop(vex::brakeType::hold);
}


void PosUpdate(){
  static float start = (vex::timer::system() / 1000.0f), end;
  static float lastTime;
  float curTime = (vex::timer::system() / 1000.0f);

  while(true){
    curTime = (vex::timer::system() / 1000.0f);
    dt = curTime - lastTime;
    lastTime = curTime;
    drivetrain.UpdatePosition(dt);

    vex::this_thread::sleep_for(20);
  }
}

void DriverLoop(){
  static Vector2 lastJoystick = Vector2(0, 0);
  controls.Set();

  if(controls.lStick.isMoved() || controls.rStick.isMoved()){
    Vector2 controllerVector(-controls.lStick.y, -controls.lStick.x);
    float angleDiff = MCEC::abs2(MCEC::AngleDiff(controllerVector.GetAngle(), lastJoystick.GetAngle()));
    float x = controls.rStick.x;
    
    drivetrain.Drive(controllerVector, x);

  }else{
    drivetrain.Stop(vex::brakeType::coast);
  }

}

void IntakeStore(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
  intakef.spin(vex::forward, 80, vex::pct);
  shoot.spin(vex::reverse, 25, vex::pct);
}
void IntakeNotStore(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  intakef.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::reverse, 25, vex::pct);
}
void IntakeGo(){
  intakef.spin(vex::forward, 80, vex::pct);
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
  shoot.spin(vex::forward, 80, vex::pct);
}
void IntakeNotGo(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  intakef.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::reverse, 80, vex::pct);
}
void IntakeStop(){
  intakef.stop();
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

  // vex::thread Thread = vex::thread(PosUpdate);

  while(1){
    DriverLoop();
    wait(20, vex::msec);
  }
}

void Auton(){
  controls.controller.Screen.clearScreen();
  // controls.controller.Screen.setCursor(1, 1);
  // controls.controller.Screen.print("Auton");

  drivetrain.points = {
    //Put the path planner points here
    {{0, 0}, 0},
    {{5, 0}, 0}
    // {{-61.634, 18.23}, 89.752},
    // {{-60.747, 20.022}, 86.875}//,
    // {{-59.861, 21.815}, 83.901},
    // {{-58.974, 23.608}, 80.817},
    // {{-58.087, 25.401}, 77.61},
    // {{-57.2, 27.193}, 74.265},
    // {{-56.314, 28.986}, 70.763},
    // {{-55.427, 30.779}, 67.077},
    // {{-54.54, 32.571}, 63.177},
    // {{-53.654, 34.364}, 59.02},
    // {{-52.767, 36.157}, 54.547},
    // {{-51.88, 37.949}, 49.672},
    // {{-50.993, 39.742}, 44.264},
    // {{-50.107, 41.535}, 38.096},
    // {{-49.22, 43.327}, 30.714},
    // {{-48.333, 45.12}, 20.865},
    // {{-47.573, 46.656}, 0},
    // {{-47.573, 46.656}, 0},
    // {{-38.706, 64.583}, 0}
  };

  drivetrain.GoToPoint();
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

  controls.B.SetOnPress(DisableForward);
  controls.B.SetOnRelease(EnableForward);

  controls.Y.SetOnPress(Auton);

  // controls.Down.SetOnPress(FaceDriver);
  // controls.Up.SetOnPress(FaceOpponent);
  // controls.Right.SetOnPress(FaceRight);
  // controls.Left.SetOnPress(FaceLeft);

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
  if(!comp.isFieldControl()){
    controls.controller.Screen.clearScreen();
    controls.controller.Screen.print("Field Control");
      comp.autonomous(Auton);
  }
}

//what does PID stand for????
//PROPORTIONAL, INTEGRAL, DERIVATIVE
float PID[] = {0.5f, 0.05f, 0.02f};

int main(){
  
  InitInertial();
  SetControls();

  drivetrain.SetRotationOffsets(
    //updated
    89.03f, //fl 3
    173.67f, //fr 9
    99.31f, //bl 6
    54.49f + 270 //br 12
  );
  drivetrain.frontLeft.SetPIDVariables (1.2f, 0.0f, 0.1f);
  drivetrain.frontRight.SetPIDVariables(1.2f, 0.0f, 0.1f);
  drivetrain.backLeft.SetPIDVariables  (1.2f, 0.0f, 0.1f);
  drivetrain.backRight.SetPIDVariables (1.2f, 0.0f, 0.1f);

  drivetrain.SetAutonPIDVariables(
    0.1f, 0.0f, 0.0f, // forward
    0.1f, 0.0f, 0.0f  // lateral
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

  controls.controller.rumble(".....");
  while(1) {
    if(!comp.isFieldControl()){
       DriverLoop();
    }
    vex::wait(10, vex::msec);
  }
}

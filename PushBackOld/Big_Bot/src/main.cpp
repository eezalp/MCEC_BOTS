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

/*--------------------
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
#include <cmath>

#define TURRET_MAX_ANGLE 23
#define TURRET_MIN_ANGLE 11

vex::inertial inertial = vex::inertial(vex::PORT9);
vex::competition comp;
vex::brain Brain;

vex::optical ballOptical = vex::optical(vex::PORT18);



MCEC::Drivetrain8 drivetrain(
    vex::PORT1, vex::PORT2, vex::PORT3, vex::PORT4, 
    vex::PORT5, vex::PORT6, vex::PORT7, vex::PORT8
);

// Motors
    vex::motor intakeMid    = vex::motor(vex::PORT12);
    vex::motor intakeFront  = vex::motor(vex::PORT10);
    vex::motor intakeBack   = vex::motor(vex::PORT11);
    vex::motor sorterMotor  = vex::motor(vex::PORT13);
    vex::motor sorterDoor   = vex::motor(vex::PORT14);
    vex::motor turretDriver = vex::motor(vex::PORT15);
    vex::motor turretRoller = vex::motor(vex::PORT16);
    vex::motor shivMotor    = vex::motor(vex::PORT17);


// Three Wires
    vex::pot turretPos = vex::pot(Brain.ThreeWirePort.A);
    vex::digital_out frontGate = vex::digital_out(Brain.ThreeWirePort.B);
    vex::digital_out turretPiston = vex::digital_out(Brain.ThreeWirePort.C);
    vex::digital_out shivPiston = vex::digital_out(Brain.ThreeWirePort.D);
// vex::controller
MCEC::Controller controls = MCEC::Controller();

#define TRACKING_WHEEL_RADIUS        1.625f // in inches
#define TRACKING_WHEEL_CIRCUMFERENCE (2 * TRACKING_WHEEL_RADIUS * M_PI) // in inches

#define IS_NOT_MINE(color) IS_RED(color)
#define MOTOR_TURRET false

float xOff = 0, yOff = 0;
float initialHeading = 0;

bool isOpen = false, isIntake = false;

enum TurretStates{LowGoal, Raising, HighGoal, Lowering};
TurretStates turretState = TurretStates::LowGoal;

void LowerTurretPnu();
void ShivUp();

void RaiseTurretPnu(){
    turretPiston.set(false);
    controls.X.SetOnPress(LowerTurretPnu);
}

void ShivDown(){
    shivPiston.set(false);
    controls.Y.SetOnPress(ShivUp);
}

void ShivUp(){
    shivPiston.set(true);
    controls.Y.SetOnPress(ShivDown);
}

void LowerTurretPnu(){
    turretPiston.set(true);
    controls.X.SetOnPress(RaiseTurretPnu);
}

void ColorDoorOpen(){
    sorterDoor.spinToPosition(5, vex::degrees, 90, vex::rpm, false);
    // sorterDoor.spinFor(vex::reverse, 360, vex::deg);
    isOpen = true;
}
void ColorDoorClose(){
    sorterDoor.spinToPosition(95, vex::degrees, 90, vex::rpm, false);
    // sorterDoor.spinFor(vex::forward, 360, vex::deg);
    isOpen = false;
}

void ColorRead(){
    if(!isIntake){ 
        ballOptical.setLight(vex::ledState::off);
        return;
    }
    ballOptical.setLight(vex::ledState::on);
    

    double hue = ballOptical.hue();
    double brightness = ballOptical.brightness();
    vex::color detectedColor = ballOptical.color();
    bool isNear = ballOptical.isNearObject();
    
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print(hue);
    Brain.Screen.print("       ");

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print(brightness);
    Brain.Screen.print("       ");

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("%X", detectedColor.rgb());
    Brain.Screen.print("       ");
    

    Brain.Screen.setCursor(6, 1);

    if(isNear){
        if(IS_NOT_MINE(detectedColor)){
            // ColorDoorOpen();
            Brain.Screen.print("Not Mine");
        }else{
            // ColorDoorClose();
            Brain.Screen.print("Mine    ");
        }

    }
}


//@param gatePos: false = open, true = closed
void IntakeGo(bool gatePos = false){
    intakeFront.spin(vex::forward, 300, vex::rpm);
    intakeBack.spin(vex::reverse, 300, vex::rpm);
    intakeMid.spin(vex::forward, 300, vex::rpm);

    turretRoller.spin(vex::forward, 300, vex::rpm);

    
    sorterMotor.spin(vex::forward, 300, vex::rpm);

    frontGate.set(gatePos);
}

void IntakeNotGo(){
    intakeFront.spin(vex::reverse, 300, vex::rpm);
    intakeBack.spin(vex::forward, 300, vex::rpm);
    intakeMid.spin(vex::reverse, 300, vex::rpm);

    turretRoller.spin(vex::reverse, 300, vex::rpm);

    sorterMotor.spin(vex::reverse, 300, vex::rpm);
}

void IntakeStop(){
    intakeFront.stop();
    intakeBack.stop();
    intakeMid.stop();

    turretRoller.stop();
    
    sorterMotor.stop();
}

void Store(){
  IntakeGo(true);
}

void Shoot(){
  IntakeGo(false);
}

void BackExit(){
    intakeFront.spin(vex::forward, 300, vex::rpm);
    intakeBack.spin(vex::reverse, 300, vex::rpm);
    intakeMid.spin(vex::forward, 300, vex::rpm);

    turretRoller.spin(vex::reverse, 300, vex::rpm);

    sorterMotor.spin(vex::reverse, 300, vex::rpm);
    ColorDoorOpen();
}

void FrontExit(){
  IntakeNotGo();
}

void DriverLoop(){
    // if(!inertial.isCalibrating()){
        controls.Set();
        if(controls.lStick.isMoved() || controls.rStick.isMoved()){
            drivetrain.Drive(-controls.lStick.y, (ABS(controls.rStick.x) > 30) ? controls.rStick.x : controls.lStick.x);
        }else{
            drivetrain.Stop();
        }

        // if(controls.r2Down){ // Intake in
        //     Store();
        //     isIntake = true;
        // }else if(controls.l2Down){ // Intake out
        //     FrontExit();
        //     isIntake = false;
        // }else if(controls.r1Down){
        //     Shoot();
        //     isIntake = false;
        // }else if(controls.l1Down){
        //     BackExit();
        //     isIntake = false;
        // }else if(!controls.r2Down && !controls.r1Down && !controls.l2Down && !controls.l1Down){
        //     IntakeStop();
        //     isIntake = false;
        // }

        ColorRead();
        drivetrain.UpdateHeading();

        // TurretUpdate();
    // }
    // vex::this_thread::sleep_for(10);
}

void Driver(){
  controls.controller.rumble("...");

  while(comp.isDriverControl()){
    DriverLoop();
  }
}

void Auton(){
  drivetrain.SetSpeed(35, vex::percentUnits::pct);

  RaiseTurretPnu();

  drivetrain.Spin((24 * (5.0f / 3.0f)) / wheelCirc);

  vex::this_thread::sleep_for(600);

  drivetrain.Rotate2(90);

  vex::this_thread::sleep_for(600);

  drivetrain.Spin((32 * (5.0f / 3.0f)) / wheelCirc);

  vex::this_thread::sleep_for(600);

  drivetrain.Rotate2(-95);

  drivetrain.Spin((5 * (5.0f / 3.0f)) / wheelCirc);

  vex::this_thread::sleep_for(600);

//   drivetrain.DriveDist(2.35, 2.35, .2f);

//   drivetrain.Rotate(270);

  vex::this_thread::sleep_for(300);
  
  Shoot();

  vex::this_thread::sleep_for(3000);

  IntakeStop();
}

void SetControls(){
    controls.X.SetOnPress(LowerTurretPnu);
    controls.Y.SetOnPress(ShivDown);

    controls.R1.SetOnPress(Shoot);
    controls.R2.SetOnPress(Store);
    controls.L1.SetOnPress(BackExit);
    controls.L2.SetOnPress(FrontExit);
    
    controls.R1.SetOnRelease(IntakeStop);
    controls.R2.SetOnRelease(IntakeStop);
    controls.L1.SetOnRelease(IntakeStop);
    controls.L2.SetOnRelease(IntakeStop);
    

    if(!comp.isFieldControl()){
        controls.Down.SetOnPress(Auton);
    }
}

int main(){
    inertial.calibrate(3);

    inertial.resetHeading();
    while(inertial.isCalibrating());
    inertial.setHeading(0, vex::deg);
    inertial.setRotation(0, vex::deg);

    ballOptical.integrationTime(50);
    ballOptical.setLightPower(100, vex::percent);
    ballOptical.setLight(vex::ledState::off);

    sorterDoor.resetPosition();

    drivetrain.SetInertial(&inertial);

    SetControls();

    controls.controller.rumble("...");

    if(comp.isFieldControl()){
      controls.controller.Screen.clearScreen();
      controls.controller.Screen.print("dwadawd");
      comp.drivercontrol(Driver);
      comp.autonomous(Auton);
    }

    while(1) {
        if(!comp.isFieldControl()){
            DriverLoop();
        }
        vex::this_thread::sleep_for(10);
    }
}

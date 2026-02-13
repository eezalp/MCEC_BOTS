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

#include <MCEC_Objects.h>

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


  void MCEC::Drivetrain::Rotate(float targ){
      uint32_t startTime = Brain.Timer.system();
      uint32_t lastTime = startTime;
      uint32_t curTime;
      float power = 0;
      float dt = 0;

      pid.Prime(_heading, targ);
      controls.controller.Screen.clearScreen();
      dt = (Brain.Timer.system() - lastTime) / 1000.0f;

      while(!pid.AtTarget(_heading, dt)){
        curTime = Brain.Timer.system();
        dt = (curTime - lastTime) / 1000.0f;

        UpdateHeading();
        power = pid.Update(_heading, dt);

        SpinR(-power);
        SpinL(power);

        controls.controller.Screen.setCursor(1, 1);
        controls.controller.Screen.print("Power: %.2f    ", power);

        lastTime = curTime;
      }

      Stop();
  }
  void MCEC::Drivetrain::Rotate(float targ, float power){
      float tVal = (targ * (M_PI/180.0f)) * (3.0f/5.0f) * wheelDistance / wheelCirc;
      float rInit = ReadRight(), lInit = ReadLeft();
      float inertialInitial = inertial->heading();

      SpinR((targ < 0) ? -power : power);
      SpinL((targ < 0) ? power : -power);
        controls.controller.Screen.clearScreen();
        controls.controller.Screen.setCursor(1, 13);
        controls.controller.Screen.print(inertialInitial);
        controls.controller.Screen.setCursor(2, 13);
        controls.controller.Screen.print(abs2(inertialInitial - inertial->heading()));
        controls.controller.Screen.setCursor(3, 13);
        controls.controller.Screen.print(abs2(targ));

      while(
        (abs2(ReadRight() - rInit) < abs2(tVal) || abs2(ReadLeft() - lInit) < abs2(tVal)) &&
        (AngleDiff(inertialInitial, inertial->heading()) < abs2(targ))
    ){
        // controls.controller.Screen.setCursor(1, 7);
        // controls.controller.Screen.print(ReadRight());
        // controls.controller.Screen.setCursor(2, 7);
        // controls.controller.Screen.print(ReadLeft());
        controls.controller.Screen.setCursor(2, 13);
        controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
      }

        controls.controller.Screen.setCursor(2, 13);
        controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
      Stop();
      inertial->setHeading(0, vex::degrees);
        controls.controller.Screen.setCursor(1, 7);
        controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
        // controls.controller.Screen.setCursor(2, 7);
        // controls.controller.Screen.print(ReadLeft());
  }
  void MCEC::Drivetrain::Spin(float revs, float power){
    float rInit = ReadRight(), lInit = ReadLeft();
    float curPow = power;
    // leftMotors.SpinTo(revs, vex::rotationUnits::rev, false);
    // rightMotors.SpinTo(revs, vex::rotationUnits::rev, true);
      while(abs2(ReadRight() - rInit) < abs2(revs) || abs2(ReadLeft() - lInit) < abs2(revs)){
        // curPow = Lerp(curPow, power, 0.075f);
        SpinR((revs < 0) ? curPow : -curPow);
        SpinL((revs < 0) ? curPow : -curPow);
        // vex::this_thread::sleep_for(10);
      }

    //   Stop();

    // controls.controller.Screen.setCursor(1, 1);
    // controls.controller.Screen.print(ReadRight());
    // controls.controller.Screen.setCursor(2, 1);
    // controls.controller.Screen.print(ReadLeft());
    // controls.controller.Screen.setCursor(3, 7);
    // controls.controller.Screen.print(revs);
  }
  void MCEC::Drivetrain::UpdateHeading(){
      static float lastR = ReadRight(), lastL = ReadLeft();
      static float prevHeading = inertial->heading();
      float dMotor = 0;
      float curR = ReadRight(), curL = ReadLeft();
      float curHeading = inertial->heading();
      float dHeading = AngleDiff(curHeading, prevHeading);

      dMotor += (wheelCirc * ((curR - lastR) - (curL - lastL))) / wheelDist;
      
      _heading += dHeading * 1.0f + dMotor * 0.0f;
      _heading = curHeading;
    controls.controller.Screen.setCursor(2, 1);
    controls.controller.Screen.print("Heading: %.2f    ", _heading);

      lastR = curR;
      lastL = curL;
      prevHeading = curHeading;
  }

void RaiseTurretPnu(){
    turretPiston.set(false);
    controls.X.SetOnPress(LowerTurretPnu);
}

void SpinRight(){
  drivetrain.SpinR(-100);
  drivetrain.SpinL(100);
  controls.controller.Screen.clearScreen();
}

void SpinLeft(){
  drivetrain.SpinR(100);
  drivetrain.SpinL(-100);
}

void ShivDown(){
    shivPiston.set(true);
    controls.Y.SetOnPress(ShivUp);
}

void ShivUp(){
    shivPiston.set(false);
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
    controls.Set();
    if(controls.lStick.isMoved() || controls.rStick.isMoved()){
        drivetrain.Drive(-controls.lStick.y, (ABS(controls.rStick.x) > 30) ? controls.rStick.x : controls.lStick.x);
    }else{
        drivetrain.Stop(vex::brakeType::coast);
    }

    ColorRead();
    drivetrain.UpdateHeading();
}

void Driver(){
  controls.controller.rumble("...");

  while(comp.isDriverControl()){
    DriverLoop();
  }
}

void Auton(){
  frontGate.set(true);
  drivetrain.inertial->setHeading(0, vex::degrees);
  drivetrain.SetSpeed(60, vex::percentUnits::pct);

  RaiseTurretPnu();

  drivetrain.Spin((12 * (5.0f / 3.0f)) / wheelCirc, 100);
//   drivetrain.Stop(vex::brakeType::hold);

  vex::this_thread::sleep_for(100);

  drivetrain.Rotate(90); // 90
  drivetrain.Stop(vex::brakeType::hold);
//   return;

  vex::this_thread::sleep_for(100);

  drivetrain.Spin((30 * (5.0f / 3.0f)) / wheelCirc, 100);
  drivetrain.Stop(vex::brakeType::hold);

  vex::this_thread::sleep_for(100);

  drivetrain.Rotate(180);
  drivetrain.Stop(vex::brakeType::hold);
  vex::this_thread::sleep_for(100);

//   Store();
//   vex::this_thread::sleep_for(2000);
//   IntakeStop();
  frontGate.set(true);

  ShivDown();

  // drivetrain.Spin((13 * (5.0f / 3.0f)) / wheelCirc, 200);
  drivetrain.SpinL(-100);
  drivetrain.SpinR(-100);
  vex::this_thread::sleep_for(1500);
  drivetrain.Stop(vex::brakeType::hold);

  vex::this_thread::sleep_for(100);
  // drivetrain.Stop();

  Store();
  for(uint8_t i = 0; i < 70; i++){
    // drivetrain.SpinL(10);
    // drivetrain.SpinR(-10);
    // drivetrain.Stop();
    vex::this_thread::sleep_for(30);
    // drivetrain.SpinL(-10);
    // drivetrain.SpinR(10);
    // drivetrain.Stop();
    vex::this_thread::sleep_for(30);
  }

  drivetrain.Stop();
  IntakeStop();
//   return;

  drivetrain.Spin((-3 * (5.0f / 3.0f)) / wheelCirc);
  drivetrain.Stop(vex::brakeType::hold);
  vex::this_thread::sleep_for(100);
  
  ShivUp();
//   return;

  drivetrain.Rotate(359.0f);
  drivetrain.Stop(vex::brakeType::hold);
  vex::this_thread::sleep_for(100);

  drivetrain.SpinL(-100);
  drivetrain.SpinR(-100);
  vex::this_thread::sleep_for(3000);
  drivetrain.Stop(vex::brakeType::hold);
  vex::this_thread::sleep_for(100);

  drivetrain.Stop();
  Shoot();

  vex::this_thread::sleep_for(600);

  while(comp.isAutonomous()){

  }

  IntakeStop();
}

void StopMoving(){
  drivetrain.Stop();
}

void SetControls(){
    controls.X.SetOnPress(LowerTurretPnu);
    controls.Y.SetOnPress(ShivDown);

    controls.R1.SetOnPress(Shoot);
    controls.R2.SetOnPress(Store);
    controls.L1.SetOnPress(BackExit);
    controls.L2.SetOnPress(FrontExit);

    controls.A.SetOnPress(SpinLeft);
    controls.B.SetOnPress(SpinRight);
    
    controls.R1.SetOnRelease(IntakeStop);
    controls.R2.SetOnRelease(IntakeStop);
    controls.L1.SetOnRelease(IntakeStop);
    controls.L2.SetOnRelease(IntakeStop);
    controls.B.SetOnRelease(StopMoving);
    controls.A.SetOnRelease(StopMoving);
    

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
    drivetrain.wheelDistance = wheelDist;
    // drivetrain.pid.SetVariables(2.5f, 0.01f, 0.22f); //P I D
    drivetrain.pid.SetVariables(2.5f, 0.0009f, 0.15f); //P I D

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

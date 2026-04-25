/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Thu Mar 21 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "MCEC_Objects.h"
// pure hatred
// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::competition comp;

  // Robot configuration code.
  vex::controller Controller1 = vex::controller(vex::primary);


  vex::inertial inertial = vex::inertial(vex::PORT9);
  MCEC::CoSwerveDrive drivetrain = MCEC::CoSwerveDrive(
    vex::PORT1, vex::PORT2, 1,
    vex::PORT3, vex::PORT4, 1,
    vex::PORT5, vex::PORT6, 1,
    vex::PORT7, vex::PORT8, 1,
    13, 13
  );

  vex::digital_out lift = vex::digital_out(Brain.ThreeWirePort.B);
  //digital_out descore = digital_out(Brain.ThreeWirePort.E);

  vex::motor intake = vex::motor(vex::PORT16, vex::ratio18_1, false);

  vex::motor lever = vex::motor(vex::PORT19, vex::ratio36_1, true);
  vex::limit leverLimit = vex::limit(Brain.ThreeWirePort.A);
  vex::limit leverLimit1 = vex::limit(Brain.ThreeWirePort.G);

  vex::motor descore = vex::motor(vex::PORT12, vex::ratio18_1);
  vex::motor matchload = vex::motor(vex::PORT11);

bool isReversed = false;
bool autonRunning = false;

void liftControl(bool on){
  lift = on;
}

void liftOn(){
  liftControl(true);
}

void liftOff(){
  liftControl(false);
}

void Undescore();

void Descore(){
  descore.spinToPosition(140, vex::degrees);
  Controller1.ButtonB.pressed(Undescore);
}
void Undescore(){
  descore.spinToPosition(175, vex::degrees);
  Controller1.ButtonB.pressed(Descore);
}

bool descoreUp = false;
void DescoreToggle(){
  descoreUp = !descoreUp;
  descore.spinToPosition(descoreUp ? 175 : 140, vex::degrees);
}

void DescoreStore(){
  descore.spinToPosition(20, vex::degrees);
}

void IntakeGo(){
  intake.spin(vex::reverse, 100, vex::pct);
}

void IntakeNotGo(){
  intake.spin(vex::fwd, 100, vex::pct);
}

void IntakeNotGoSlo(){
  intake.spin(vex::fwd, 40, vex::pct);
}

void IntakeStop(){
  intake.stop();
}

void InvertControls(){
  isReversed = !isReversed;
}

void shoot(){
  lever.resetPosition();
  IntakeGo();
  while(!leverLimit.pressing() && !leverLimit1.pressing()){
    if(!lift)
        lever.spin(vex::reverse, 50, vex::pct);
    else
        lever.spin(vex::reverse, 100, vex::pct);
  }
  lever.stop();
  IntakeStop();

  double distance = lever.position(vex::degrees);

    //lever.spinFor(fwd, -distance * 1.045, degrees, true);
   lever.spin(vex::fwd, 100, vex::pct);
  while((abs(lever.velocity(vex::pct) > 0.5) || abs(lever.position(vex::degrees)/distance) > 0.5f)  && !Controller1.ButtonX.pressing()){
    lever.spin(vex::fwd, 100, vex::pct);
  }
  lever.stop();

}

void DriverLoop(){
  Brain.Screen.render();
  if(autonRunning) return;
  int controllerX = Controller1.Axis4.position(); //* 2;
  int controllerY = Controller1.Axis3.position(); //* 2;
  int turning = Controller1.Axis1.position(); //* 2;

  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) controllerX = 0;
  if(abs(Controller1.Axis3.position()) < 5) controllerY = 0;
  if(abs(Controller1.Axis1.position()) < 5) turning = 0;

  // double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  // double theta = atan2(controllerY, controllerX);
  // double theta2 = theta - (inertial.angle() * (M_PI/180));
  // double x2 = magnitude * cos(theta2) * speed;
  // double y2 = magnitude * sin(theta2) * speed;

  //inertial reset
  // if(Controller1.ButtonB.pressing()) {
  //   inertial.resetRotation();
  // }

  //Motor output
  // double leftF = y2 + x2 + turning;
  // double leftB = y2 - x2 + turning;
  // double rightF = y2 - x2 - turning;
  // double rightB = y2 + x2 - turning;


  //turning set to 35% at driver request
  float scaleFactor = Controller1.ButtonR2.pressing() ? 0.5f : 1;

  wait(20, vex::msec);
}

void Driver(){
  Controller1.rumble(".");

  while(comp.isDriverControl()){
    DriverLoop();
  }
}

void LeverDown(){
  lever.spin(vex::forward, 100, vex::pct);
}
void LeverStop(){
  lever.stop();
}

void Auton(){
  autonRunning = true;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton Go");

  autonRunning = false;
}

void SetupFieldControl(){
  if(comp.isFieldControl()){
    comp.drivercontrol(Driver);
    comp.autonomous(Auton);
  }
}

void DeployMatchlaod(){
  matchload.spinToPosition(20, vex::degrees);
}

void StoreMatchload(){
  matchload.spinToPosition(0, vex::degrees);
}

void SetupControls(){
  Controller1.ButtonLeft.pressed(DescoreStore);
  Controller1.ButtonRight.pressed(Undescore);
  Controller1.ButtonUp.pressed(liftOn);
  Controller1.ButtonDown.pressed(liftOff);

  Controller1.ButtonB.pressed(DescoreToggle);
  Controller1.ButtonA.pressed(shoot);
  Controller1.ButtonY.pressed(InvertControls);
  Controller1.ButtonX.pressed(LeverDown);
  
  Controller1.ButtonX.released(LeverStop);

  Controller1.ButtonL2.pressed(IntakeNotGoSlo);
  Controller1.ButtonL1.pressed(IntakeNotGo);
  Controller1.ButtonR1.pressed(IntakeGo);

  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR1.released(IntakeStop);
  Controller1.ButtonL2.released(IntakeStop);
}

void Setupinertial(){
  inertial.calibrate(3);
  inertial.resetHeading();
  while(inertial.isCalibrating());
  inertial.setHeading(0, vex::deg);
  inertial.setRotation(0, vex::deg);

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("inertial Calibrated");
}

int main() {
  int count = 0;

  SetupFieldControl();

  Setupinertial();

  SetupControls();

//   vex::Gif gif("world.gif", 200, 0 );
//   vex::Gif gif1 ("hapy.gif", 0, 0);

  lever.setStopping(vex::brakeType::hold);

  descore.resetPosition();
  descore.spinToPosition(175, vex::degrees);
  DescoreStore();
  matchload.setPosition(20, vex::rotationUnits::rev);
  
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
  }
}
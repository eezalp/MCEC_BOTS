/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Thu Mar 21 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
#include "niicec.h"

using namespace vex;



// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::competition comp;

  // Robot configuration code.
  controller Controller1 = controller(primary);



  XDrive xdrive (9, 12, 16, 17, 14, ratio18_1);

  digital_out lift = digital_out(Brain.ThreeWirePort.B);
  //digital_out descore = digital_out(Brain.ThreeWirePort.E);



  motor intake = motor(PORT16, ratio18_1, false);

  motor lever = motor(PORT19, ratio36_1, true);
  limit leverLimit = limit(Brain.ThreeWirePort.A);
  limit leverLimit1 = limit(Brain.ThreeWirePort.G);

  motor descore = motor(PORT12, ratio18_1);

bool isReversed = false;

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
  descore.spinToPosition(140, degrees);
  Controller1.ButtonB.pressed(Undescore);
}
void Undescore(){
  descore.spinToPosition(175, degrees);
  Controller1.ButtonB.pressed(Descore);
}

bool descoreUp = false;
void DescoreToggle(){
  descoreUp = !descore;
  descore.spinToPosition(descoreUp ? 175 : 140, degrees);
}

void DescoreStore(){
  descore.spinToPosition(20, degrees);
}

void IntakeGo(){
  intake.spin(reverse, 100, pct);
}

void IntakeNotGo(){
  intake.spin(fwd, 100, pct);
}

void IntakeNotGoSlo(){
  intake.spin(fwd, 40, pct);
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
        lever.spin(reverse, 50, pct);
    else
        lever.spin(reverse, 100, pct);
  }
  lever.stop();
  IntakeStop();

  double distance = lever.position(degrees);
//   Controller1.Screen.setCursor(1,1);
//   Controller1.Screen.print(distance);

    //lever.spinFor(fwd, -distance * 1.045, degrees, true);
   lever.spin(fwd, 100, pct);
  while(abs(lever.velocity(pct) > 0.5) || abs(lever.position(degrees)/distance) > 0.5f  && !Controller1.ButtonX.pressing()){
    lever.spin(fwd, 100, pct);
  }
  lever.stop();

}

void DriverLoop(){
  Brain.Screen.render();

  int controllerX = Controller1.Axis4.position(); //* 2;
  int controllerY = Controller1.Axis3.position(); //* 2;
  int turning = Controller1.Axis1.position(); //* 2;

  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) controllerX = 0;
  if(abs(Controller1.Axis3.position()) < 5) controllerY = 0;
  if(abs(Controller1.Axis1.position()) < 5) turning = 0;

  // double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  // double theta = atan2(controllerY, controllerX);
  // double theta2 = theta - (imu.angle() * (M_PI/180));
  // double x2 = magnitude * cos(theta2) * speed;
  // double y2 = magnitude * sin(theta2) * speed;

  //Imu reset
  // if(Controller1.ButtonB.pressing()) {
  //   imu.resetRotation();
  // }

  //Motor output
  // double leftF = y2 + x2 + turning;
  // double leftB = y2 - x2 + turning;
  // double rightF = y2 - x2 - turning;
  // double rightB = y2 + x2 - turning;

//   int leftF = controllerY + controllerX + turning;
//   int leftB = controllerY - controllerX + turning;
//   int rightF = controllerY - controllerX - turning;
//   int rightB = controllerY + controllerX - turning;

  xdrive.setTarget((isReversed ? -1 : 1) * controllerX, (isReversed ? -1 : 1) * controllerY, turning);


Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(leverLimit1.pressing());

  wait(20, msec);
}

void Driver(){
  Controller1.rumble(".");

  while(comp.isDriverControl()){
    DriverLoop();
  }
}

void LeverDown(){
  lever.spin(forward, 100, pct);
}
void LeverStop(){
  lever.stop();
}

void Auton(){

}

void SetupFieldControl(){
  if(comp.isFieldControl()){
    comp.drivercontrol(Driver);
    comp.autonomous(Auton);
  }
}

int main() {
  int count = 0;

  SetupFieldControl();

//   vex::Gif gif("world.gif", 200, 0 );
//   vex::Gif gif1 ("hapy.gif", 0, 0);

  Controller1.ButtonLeft.pressed(DescoreStore);
  Controller1.ButtonRight.pressed(Undescore);
  Controller1.ButtonB.pressed(DescoreToggle);


  Controller1.ButtonA.pressed(shoot);
  Controller1.ButtonY.pressed(InvertControls);

  Controller1.ButtonL2.pressed(IntakeNotGoSlo);
  Controller1.ButtonR2.pressed(IntakeGo);
  Controller1.ButtonL1.pressed(IntakeNotGo);
  Controller1.ButtonR1.pressed(IntakeGo);

  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR1.released(IntakeStop);
  Controller1.ButtonL2.released(IntakeStop);
  Controller1.ButtonR2.released(IntakeStop);

  Controller1.ButtonUp.pressed(liftOn);
  Controller1.ButtonDown.pressed(liftOff);
  
  Controller1.ButtonX.pressed(LeverDown);
  Controller1.ButtonX.released(LeverStop);

  lever.setStopping(brake);

  descore.resetPosition();
  descore.spinToPosition(175, degrees);

  
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
  }
}
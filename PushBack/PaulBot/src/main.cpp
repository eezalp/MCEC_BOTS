/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Thu Mar 21 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "gifclass.h"
#include <cmath>

using namespace vex;
#define speed 20

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::competition comp;

  // Robot configuration code.
  controller Controller1 = controller(primary);

  inertial imu = inertial(PORT10);

  motor Fright = motor(PORT13, ratio18_1, true);

  motor Bright = motor(PORT14, ratio18_1, true);

  motor Fleft = motor(PORT12, ratio18_1, false);

  motor Bleft = motor(PORT15, ratio18_1, false);

  digital_out lift = digital_out(Brain.ThreeWirePort.D);
  digital_out descore = digital_out(Brain.ThreeWirePort.E);

  motor intake = motor(PORT16, ratio18_1, false);

  motor lever = motor(PORT11, ratio36_1, true);
  limit leverLimit = limit(Brain.ThreeWirePort.G);


void setVel(double LF, double LB, double RF, double RB){
  Fleft.spin(fwd, LF, rpm);
  Bleft.spin(fwd, LB, rpm);
  Fright.spin(fwd, RF, rpm);
  Bright.spin(fwd, RB, rpm);
}

void liftControl(bool on){
  lift = on;
}

void liftOn(){
  liftControl(true);
}

void liftOff(){
  liftControl(false);
}

void Descore(){
  descore = true;
}
void Undescore(){
  descore = false;
}

void IntakeGo(){
  intake.spin(reverse, 100, pct);
}

void IntakeNotGo(){
  intake.spin(fwd, 100, pct);
}

void IntakeStop(){
  intake.stop();
}

void shoot(){
  lever.resetPosition();
  IntakeGo();
  while(!leverLimit.pressing() && !Controller1.ButtonY.pressing()){
    lever.spin(reverse, 100, pct);
  }
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(leverLimit.pressing());
  lever.stop();
  IntakeStop();

  double distance = lever.position(degrees);
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(distance);

  lever.spinFor(fwd, -distance*1.025, degrees, false);
  // lever.spin(fwd, 100, pct);
  // while(lever.efficiency() > 5 && !Controller1.ButtonY.pressing()){
  //   lever.spin(fwd, 100, pct);
  // }
  // lever.stop();

}

void Stop(){
  Fleft.stop(vex::brakeType::hold);
  Bleft.stop(vex::brakeType::hold);
  Fright.stop(vex::brakeType::hold);
  Bright.stop(vex::brakeType::hold);
}

void DriverLoop(){
  Brain.Screen.render();

  int controllerX = -Controller1.Axis4.position();// * 2;
  int controllerY = -Controller1.Axis3.position();// * 2;
  int turning = Controller1.Axis1.position();// * 2;

  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) {controllerX = 0; Stop();}
  if(abs(Controller1.Axis3.position()) < 5) {controllerY = 0; Stop();}
  if(abs(Controller1.Axis1.position()) < 5) {turning = 0; Stop();}

  // double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  // double theta = atan2(controllerY, controllerX);
  // double theta2 = theta - (imu.angle() * (M_PI/180));
  // double x2 = magnitude * cos(theta2) * speed;
  // double y2 = magnitude * sin(theta2) * speed;

  Controller1.Screen.setCursor(1,1);
  // Controller1.Screen.print(leverLimit.pressing());

  //Imu reset
  // if(Controller1.ButtonB.pressing()) {
  //   imu.resetRotation();
  // }

  //Motor output
  // double leftF = y2 + x2 + turning;
  // double leftB = y2 - x2 + turning;
  // double rightF = y2 - x2 - turning;
  // double rightB = y2 + x2 - turning;

  int leftF = controllerY + controllerX + turning;
  int leftB = controllerY - controllerX + turning;
  int rightF = controllerY - controllerX - turning;
  int rightB = controllerY + controllerX - turning;


  setVel(leftF * 4, leftB * 4, rightF * 4, rightB * 4);

  Controller1.ButtonUp.pressed(liftOn);
  Controller1.ButtonDown.pressed(liftOff);
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

  vex::Gif gif("world.gif", 200, 0 );
  vex::Gif gif1 ("hapy.gif", 0, 0);

  Controller1.ButtonLeft.pressed(Descore);
  Controller1.ButtonRight.pressed(Undescore);
  Controller1.ButtonA.pressed(shoot);
  Controller1.ButtonL2.pressed(IntakeNotGo);
  Controller1.ButtonR2.pressed(IntakeGo);
  Controller1.ButtonL2.released(IntakeStop);
  Controller1.ButtonR2.released(IntakeStop);
  
  Controller1.ButtonX.pressed(LeverDown);
  Controller1.ButtonX.released(LeverStop);

  lever.setStopping(brake);
  
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
  }
}
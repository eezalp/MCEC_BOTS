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

using namespace vex;// pure hatred

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
vex::competition comp;

  // Robot configuration code.
  controller Controller1 = controller(primary);


  inertial imu =  inertial(PORT9);
  XDrive xdrive (&imu, 12, 16, 17, 14, ratio18_1);

  digital_out lift = digital_out(Brain.ThreeWirePort.B);
  //digital_out descore = digital_out(Brain.ThreeWirePort.E);

  motor intake = motor(PORT16, ratio18_1, false);

  motor lever = motor(PORT19, ratio36_1, true);
  limit leverLimit = limit(Brain.ThreeWirePort.A);
  limit leverLimit1 = limit(Brain.ThreeWirePort.G);

  motor descore = motor(PORT12, ratio18_1);

  motor gorilla = motor(PORT14, ratio18_1);

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
  descoreUp = !descoreUp;
  descore.spinToPosition(descoreUp ? 175 : 140, degrees);
}

void DescoreStore(){
  descore.spinToPosition(20, degrees);
}

float gDistance = 0;
void gorillaDown(){
  gorilla.resetPosition();
  gorilla.spin(reverse);
  wait(100, msec);
  while(abs(gorilla.velocity(pct)) > 0.5  && !Controller1.ButtonX.pressing()){
  }
  gorilla.stop();
  gDistance = gorilla.position(degrees);
}

void gorillaUp(){
  gorilla.spin(fwd);
  wait(100, msec);
  while(abs(gorilla.velocity(pct)) > 0.5  && !Controller1.ButtonX.pressing()){
   }
  gorilla.stop();
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
  while((abs(lever.velocity(pct) > 0.5) || abs(lever.position(degrees)/distance) > 0.5f)  && !Controller1.ButtonX.pressing()){
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

//turning set to 35% at driver request
  float scaleFactor = Controller1.ButtonR2.pressing() ? 0.5f : 1;
  xdrive.setTarget((isReversed ? -1 : 1) * controllerX * scaleFactor, (isReversed ? -1 : 1) * controllerY * scaleFactor, turning*0.6f);


  // Controller1.Screen.setCursor(2,1);
  // Controller1.Screen.print("%0.2f  %0.2f ", imu.orientation(orientationType::roll, rotationUnits::deg), imu.orientation(orientationType::pitch, rotationUnits::deg));
  // Controller1.Screen.setCursor(3,1);
  // Controller1.Screen.print("%0.2f   ", imu.orientation(orientationType::yaw, rotationUnits::deg));

  xdrive.UpdateMotorSpeeds();

  

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

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Auton Go");

  //16 pts
  // //approach goal
  // xdrive.move(2.1, RIGHT);
  // xdrive.move(22.1);
  // //turn to face goal
  // xdrive.turnInPlace(45);
  // shoot();
  // xdrive.move(5, BACKWARD);
  
  // //
  // xdrive.turnInPlace(-40);
  // IntakeGo();
  // xdrive.move(40, BACKWARD);
  // wait(1, seconds);
  // IntakeStop();


  //other approach
  xdrive.move(5, RIGHT);
  IntakeGo();
  xdrive.move(25);
  xdrive.turnInPlace(90);
  xdrive.move(10);
  xdrive.turnInPlace(-90);
  xdrive.move(5, BACKWARD);
  IntakeStop();

  xdrive.turnInPlace(-45);
  xdrive.move(10);

}

void SetupFieldControl(){
  comp.drivercontrol(Driver);
  comp.autonomous(Auton);
}

void SetupControls(){
  Controller1.ButtonLeft.pressed(DescoreStore);
  Controller1.ButtonRight.pressed(Undescore);
  Controller1.ButtonB.pressed(DescoreToggle);


  Controller1.ButtonA.pressed(shoot);
  Controller1.ButtonY.pressed(InvertControls);

  Controller1.ButtonL2.pressed(IntakeNotGoSlo);
  Controller1.ButtonL1.pressed(IntakeNotGo);
  Controller1.ButtonR1.pressed(IntakeGo);

  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR1.released(IntakeStop);
  Controller1.ButtonL2.released(IntakeStop);

  Controller1.ButtonUp.pressed(gorillaUp);
  Controller1.ButtonDown.pressed(gorillaDown);
  
  Controller1.ButtonX.pressed(LeverDown);
  Controller1.ButtonX.released(LeverStop);
}

void SetupImu(){
  imu.calibrate(3);
  imu.resetHeading();
  while(imu.isCalibrating());
  imu.setHeading(0, vex::deg);
  imu.setRotation(0, vex::deg);

  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1,1);
  // Controller1.Screen.print("Imu Calibrated");
}

int main() {
  int count = 0;

  SetupFieldControl();

  SetupImu();

  SetupControls();

//   vex::Gif gif("world.gif", 200, 0 );
//   vex::Gif gif1 ("hapy.gif", 0, 0);

  lever.setStopping(brake);

  descore.resetPosition();
  descore.spinToPosition(175, degrees);

  
  while(1) {
    if(!comp.isFieldControl() && !comp.isAutonomous() && !comp.isDriverControl()){
      DriverLoop();
    }
    wait(20, msec);
  }
}
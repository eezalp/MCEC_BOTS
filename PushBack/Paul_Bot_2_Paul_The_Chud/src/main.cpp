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


  //TODO:
  //Change lever to 1 motor C
  //Toggleable between field and robot oriented driving



  inertial imu =  inertial(PORT5);

  //THESE ARE INTSSSSS THEY ARE INDEXED FROM ZERO
  //HELP ME HEEEEELP MEEEEE
  XDrive xdrive (&imu, 0, 19, 9, 10, ratio6_1);

  motor intake = motor(PORT2, ratio18_1, false);
  //#ratio doesn't matter if you use pct always
  motor scoring = motor(PORT9, ratio18_1, true);

  // motor lever = motor(PORT19, ratio36_1, true);
  // limit leverLimit = limit(Brain.ThreeWirePort.A);


  // limit leverLimit1 = limit(Brain.ThreeWirePort.G);

  motor descore = motor(PORT12, ratio18_1);
  // motor matchload = motor(PORT11);

bool isReversed = false;
bool autonRunning = false;

// void liftControl(bool on){
//   lift = on;
// }

// void liftOn(){
//   liftControl(true);
// }

// void liftOff(){
//   liftControl(false);
// }

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

void IntakeGo(){
  intake.spin(reverse, 100, pct);
  scoring.spin(reverse, 100, pct);
}

void IntakeStore(){
  scoring.spin(reverse, 100, pct);
}

void IntakeNotGo(){
  intake.spin(fwd, 100, pct);
  scoring.spin(fwd, 100, pct);
}

void IntakeNotGoSlo(){
  scoring.spin(fwd, 40, pct);
}

void IntakeStop(){
  intake.stop();
  scoring.stop();
}

void InvertControls(){
  isReversed = !isReversed;
}

// void shoot(){
//   lever.resetPosition();
//   IntakeGo();
//   while(!leverLimit.pressing() && !leverLimit1.pressing()){
//     if(!lift)
//         lever.spin(reverse, 50, pct);
//     else
//         lever.spin(reverse, 100, pct);
//   }
//   lever.stop();
//   IntakeStop();

//   double distance = lever.position(degrees);

//     //lever.spinFor(fwd, -distance * 1.045, degrees, true);
//    lever.spin(fwd, 100, pct);
//   while((abs(lever.velocity(pct) > 0.5) || abs(lever.position(degrees)/distance) > 0.5f)  && !Controller1.ButtonX.pressing()){
//     lever.spin(fwd, 100, pct);
//   }
//   lever.stop();

// }

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

// void LeverDown(){
//   lever.spin(forward, 100, pct);
// }
// void LeverStop(){
//   lever.stop();
// }

void Auton(){
  // autonRunning = true;
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1,1);
  // Controller1.Screen.print("Auton Go");

  // // xdrive.move(5.0f, RIGHT);
  // // IntakeGo();
  // // xdrive.move(25.0f);
  // // xdrive.turnInPlace(90.0f);
  // // xdrive.move(10.0f);
  // // xdrive.turnInPlace(-90.0f);
  // // xdrive.move(5.0f, BACKWARD);
  // // IntakeStop();

  // // xdrive.turnInPlace(-45.0f);
  // // xdrive.move(10.0f);

  // //approach goal
  // xdrive.move(2.1, RIGHT);
  // xdrive.move(22.1);
  // //turn to face goal
  // xdrive.turnInPlace(43.5f);
  // xdrive.move(5.9, FORWARD);
  // //shoot();
  // xdrive.move(10.85, BACKWARD);
  
  // //
  // xdrive.turnInPlace(-40.0f);
  // xdrive.move(8, BACKWARD);
  // xdrive.move(0.4, LEFT);
  // IntakeNotGo();
  // xdrive.move(100, BACKWARD, 2.0f);
  // wait(20, seconds);
  // IntakeStop();
  
  // // //start
  // // xdrive.move(0, LEFT);
  // // xdrive.move(13, BACKWARD);
  // // xdrive.turnInPlace(47.8f-90);
  // // this_thread::sleep_for(1000);
  // // IntakeGo();
  // // xdrive.move(22.75, BACKWARD);

  // // vex::this_thread::sleep_for(500);
  
  // // //edge ball on the left
  // // xdrive.move(13, FORWARD);
  // // IntakeStop();
  // // xdrive.move(10.5, RIGHT);
  // // xdrive.move(4, BACKWARD);
  // // IntakeGo();
  // // vex::this_thread::sleep_for(500);

  // // //first alligning
  // // xdrive.move(2, LEFT);
  // // xdrive.move(20, FORWARD);
  // // IntakeStop();
  // // xdrive.move(12,RIGHT);
  // // xdrive.move(2.3f, BACKWARD);

  // // //first deposit
  // // IntakeNotGoSlo();
  // // vex::this_thread::sleep_for(5000);

  // // xdrive.move(2, BACKWARD);
  // // xdrive.move(5, FORWARD);
  // // IntakeStop();

  // // //edge ball on the right
  // // xdrive.move(10, FORWARD);
  // // xdrive.turnInPlace(0.0f);
  // // IntakeGo();
  // // xdrive.move(14, BACKWARD);
  // // this_thread::sleep_for(500);
  // // IntakeStop();

  // // //two red balls all the way on the right
  // // xdrive.move(6, FORWARD);
  // // xdrive.turnInPlace(47.86f);
  // // xdrive.move(8, FORWARD);



  // // end
  // // start of plan https://www.desmos.com/calculator/peixhszlp3
  // // xdrive.turnInPlace(90 - 48.65f);
  // // xdrive.move(33.3f, BACKWARD);
  
  // // xdrive.turnInPlace(53.57f + 180);
  // // xdrive.move(52.2f, BACKWARD);
  
  // // xdrive.turnInPlace(-90.0f + 180);
  // // xdrive.move(116.0f, BACKWARD);
  
  // // xdrive.turnInPlace(112.7f + 180);
  // // xdrive.move(80.23f, BACKWARD);
  
  // // xdrive.turnInPlace(180.0f + 180);
  // // xdrive.move(56.0f, BACKWARD);
  
  // // xdrive.turnInPlace(-146.3f + 180);
  // // xdrive.move(28.84f, BACKWARD);
  autonRunning = false;
}

void SetupFieldControl(){
  comp.drivercontrol(Driver);
  comp.autonomous(Auton);
}

// void DeployMatchlaod(){
//   matchload.spinToPosition(20, degrees);
// }

// void StoreMatchload(){
//   matchload.spinToPosition(0, degrees);
// }

void SetupControls(){
  Controller1.ButtonLeft.pressed(DescoreStore);
  Controller1.ButtonRight.pressed(Undescore);
  // Controller1.ButtonUp.pressed(liftOn);
  // Controller1.ButtonDown.pressed(liftOff);

  Controller1.ButtonB.pressed(DescoreToggle);
  // Controller1.ButtonA.pressed(shoot);
  Controller1.ButtonY.pressed(InvertControls);
  // Controller1.ButtonX.pressed(LeverDown);
  
  // Controller1.ButtonX.released(LeverStop);

  Controller1.ButtonL2.pressed(IntakeNotGoSlo);
  Controller1.ButtonL1.pressed(IntakeNotGo);
  Controller1.ButtonR1.pressed(IntakeGo);
  Controller1.ButtonR2.pressed(IntakeStore);

  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR1.released(IntakeStop);
  Controller1.ButtonL2.released(IntakeStop);
  Controller1.ButtonR2.released(IntakeStop);
}

void SetupImu(){
  imu.calibrate(3);
  imu.resetHeading();
  while(imu.isCalibrating());
  imu.setHeading(0, vex::deg);
  imu.setRotation(0, vex::deg);

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Imu Calibrated");
}

int main() {
  int count = 0;

  SetupFieldControl();

  SetupImu();

  SetupControls();

//   vex::Gif gif("world.gif", 200, 0 );
//   vex::Gif gif1 ("hapy.gif", 0, 0);

  // lever.setStopping(brake);

  descore.resetPosition();
  descore.spinToPosition(175, degrees);
  DescoreStore();
  // matchload.setPosition(20, vex::rotationUnits::rev);

  // xdrive.fieldOriented = true;
  
  while(1) {
    if(!comp.isFieldControl()){
      DriverLoop();
    }
  }
}
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nichomac                                                  */
/*    Created:      2/12/2026, 9:41:47 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

}



void autonomous(void) {

}


brain Brain;
controller Controller1;


// --- Swerve Module ---
class SwerveModule {
    const int REVERSE_ENTER = 100;
    const int REVERSE_EXIT  = 80;
    motor &topMotor;
    motor &bottomMotor;
    rotation &rotSensor;
    
    double kP = 0.5;
    double kD = 0.1;
    double prevError = 0;
    bool reversed = false, onShortest = false;

public:
  float offset = 0;
    SwerveModule(motor &top, motor &bot, rotation &rot)
        : topMotor(top), bottomMotor(bot), rotSensor(rot) {}

    double angleDiff(double target, double current) {
        double diff = fmod(target - current + 360.0, 360.0);
        if (diff > 180.0) diff -= 360.0;
        return diff;
    }

    void set(double driveSpeed, double targetAngleDeg) {
        double currentAngle = fmod(rotSensor.position(degrees) + offset, 360.0);
        if (currentAngle < 0) currentAngle += 360.0;

        double error = angleDiff(targetAngleDeg, currentAngle);
        
        // if(fabs(error) > REVERSE_ENTER && !reversed){
        //   reversed = true;
        // }else if (fabs(error) < REVERSE_EXIT && reversed){
        //   reversed = false;
        // }
        // if(reversed){
        //   driveSpeed = -driveSpeed;
        //   error = angleDiff(targetAngleDeg + 180.0, currentAngle);
        // }

        //reverse drive instead of rotating > 90°
        // if (fabs(error) > 99.0) {  // give some slack
        //     error = angleDiff(targetAngleDeg + 180.0, currentAngle);
        //     driveSpeed = -driveSpeed;
        // }

        // PD controller for steering
        double steerOutput = kP * error + kD * (error - prevError);
        prevError = error;
        
        //drive slower when the wheels are not yet aligned
        double alignmentFactor = cos(error * M_PI / 180.0);
        driveSpeed *= fmax(alignmentFactor, 0.0);

        // Combine drive + steer into two motor powers
        double motor1 = driveSpeed * 100.0 + steerOutput;
        double motor2 = driveSpeed * 100.0 - steerOutput;

        // Clamp to [-100, 100]
        double maxVal = fmax(fabs(motor1), fabs(motor2));
        if (maxVal > 100.0) {
            motor1 = motor1 / maxVal * 100.0;
            motor2 = motor2 / maxVal * 100.0;
        }

        topMotor.spin(forward, motor1, percent);
        bottomMotor.spin(forward, motor2, percent);
    }

    void stop() {
        topMotor.stop(brake);
        bottomMotor.stop(brake);
    }
};


motor FL_top(PORT1, ratio18_1, false);
motor FL_bot(PORT2, ratio18_1, false); 
rotation FL_rot(PORT3);

motor FR_top(PORT7, ratio18_1, true);
motor FR_bot(PORT8, ratio18_1, true);
rotation FR_rot(PORT9);

motor BL_top(PORT4, ratio18_1, false);
motor BL_bot(PORT5, ratio18_1, false);
rotation BL_rot(PORT6);

motor BR_top(PORT10, ratio18_1, false);
motor BR_bot(PORT11, ratio18_1, false);
rotation BR_rot(PORT12);

SwerveModule FL(FL_top, FL_bot, FL_rot);
SwerveModule FR(FR_top, FR_bot, FR_rot);
SwerveModule BL(BL_top, BL_bot, BL_rot);
SwerveModule BR(BR_top, BR_bot, BR_rot);

digital_out turret(Brain.ThreeWirePort.B);

motor intakeb = motor(PORT17);
motor intakem = motor(PORT19);
motor shoot   = motor(PORT20); //shooter

// Module positions relative to center (inches)
float dist = 20.25f - 2 * (2.708f);
double modX[] = {-dist, dist, -dist, dist};  // FL, FR, BL, BR
double modY[] = {dist, dist, -dist, -dist};

void calibrate() {
    FL_rot.setPosition(0, degrees);
    FR_rot.setPosition(0, degrees);
    BL_rot.setPosition(0, degrees);
    BR_rot.setPosition(0, degrees);

    Brain.Screen.print("Calibrated!");
}

void swerveDrive(double vx, double vy, double omega) {
    SwerveModule* modules[] = {&FL, &FR, &BL, &BR};
    double speeds[4], angles[4];
    double maxSpeed = 0;

    for (int i = 0; i < 4; i++) {
        double rx = -omega * modY[i];
        double ry = omega * modX[i];

        double totalX = vx + rx;
        double totalY = vy + ry;

        speeds[i] = sqrt(totalX * totalX + totalY * totalY);
        angles[i] = (atan2(totalX, totalY) * 180.0 / M_PI) + 90;

        if (speeds[i] > maxSpeed) maxSpeed = speeds[i];
    }

    // Normalize speeds if any exceed 1.0
    if (maxSpeed > 1.0) {
        for (int i = 0; i < 4; i++) speeds[i] /= maxSpeed;
    }

    // Deadband — don't steer if barely moving
    if (maxSpeed < 0.05) {
        for (int i = 0; i < 4; i++) modules[i]->stop();
        return;
    }

    for (int i = 0; i < 4; i++) {
        modules[i]->set(speeds[i], angles[i]);
    }
}

void IntakeStore(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
}
void IntakeNotStore(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
}
void IntakeGo(){
  intakeb.spin(vex::forward, 80, vex::pct);
  intakem.spin(vex::forward, 80, vex::pct);
  shoot.spin(vex::reverse, 80, vex::pct);
}
void IntakeNotGo(){
  intakeb.spin(vex::reverse, 80, vex::pct);
  intakem.spin(vex::reverse, 80, vex::pct);
  shoot.spin(vex::forward, 80, vex::pct);
}
void IntakeStop(){
  intakeb.stop();
  intakem.stop();
  shoot.stop();
}

void TurretDown();

void TurretUp(){
    turret.set(true);
    Controller1.ButtonA.pressed(TurretDown);
}
void TurretDown(){
    turret.set(false);
    Controller1.ButtonA.pressed(TurretUp);
}

void setcontrols(){
  Controller1.ButtonR1.pressed(IntakeGo);
  Controller1.ButtonL1.pressed(IntakeNotGo);
  Controller1.ButtonR2.pressed(IntakeStore);
  Controller1.ButtonL2.pressed(IntakeNotStore);

  Controller1.ButtonR1.released(IntakeStop);
  Controller1.ButtonL1.released(IntakeStop);
  Controller1.ButtonR2.released(IntakeStop);
  Controller1.ButtonL2.released(IntakeStop);
}

void usercontrol(void) {

  while (1) {

    double vx    = Controller1.Axis4.position() / 100.0;  // left stick X
    double vy    = Controller1.Axis3.position() / 100.0;  // left stick Y
    double omega = Controller1.Axis1.position() / 100.0;  // right stick X

    // Deadband on joysticks
    if (fabs(vx) < 0.05) vx = 0;
    if (fabs(vy) < 0.05) vy = 0;
    if (fabs(omega) < 0.05) omega = 0;

    swerveDrive(vx, vy, omega);

    if(Controller1.ButtonA.pressing() && Controller1.ButtonB.pressing()){
      calibrate();
    }

        Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("FL: %.1f", FL_rot.position(degrees));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("FR: %.1f", FR_rot.position(degrees));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("BL: %.1f", BL_rot.position(degrees));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("BR: %.1f", BR_rot.position(degrees));


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  BL.offset = 180;
  // FR.offset = 180;

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

#include "MCEC_Objects.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}
namespace MCEC{

// Drivetrain

  void Drivetrain::Drive(int joyX, int joyY){
    // Ensure the joystick inputs are within the valid range
    if(ABS(joyX) > 100 || ABS(joyY) > 100){
        return;
    }

    // Apply a deadzone to the joystick inputs
    #define DEADZONE 5
    joyX = (ABS(joyX) < DEADZONE) ? 0 : joyX;
    joyY = (ABS(joyY) < DEADZONE) ? 0 : joyY;

    // Setup the influence of each joystick axis
    float totalPower = ABS(joyX) + ABS(joyY);
    // Handle case where joysticks are both pointed in the wrong axis
    if(totalPower == 0){
        return;
    } 
    float py = (joyY / 100.0f) * (joyY < 0 ? -1 : 1);
    float px = (joyX / 100.0f) * (joyX < 0 ? -1 : 1);
    float tpx = (joyX / totalPower) * (joyX < 0 ? -1 : 1);
    float tpy = (joyY / totalPower) * (joyY < 0 ? -1 : 1);

    // Apply a scaling factor to each axis
    float xPower = (joyX * 6 * px * tpx);
    float yPower = (joyY * 6 * py * tpy);

    Lpower = (xPower + yPower);
    Rpower = (xPower - yPower);

    // Apply the power to the motors
    ApplyPower(Lpower, Rpower);
}
  void Drivetrain::SetInertial(vex::inertial* _inertial){
    inertial = _inertial;
  }
  void Drivetrain::UpdateHeading(){
      static float headingMotor = 0;
      static int16_t lastR = ReadRight(), lastL = ReadLeft();
      int16_t curR = ReadRight(), curL = ReadLeft();

      headingMotor += (wheelCirc * ((curR - lastR) - (curL - lastL)) / wheelDist);
      
      _heading = inertial->heading();// * 0.8f + headingMotor * 0.2f;


      lastR = curR;
      lastL = curL;
  }
  void Drivetrain::Rotate(float targ){
      float tVal = (targ / 2) * wheelDist;
      float rInit = ReadRight(), lInit = ReadLeft();

      SpinR((targ < 0) ? -60 : 60);
      SpinL((targ < 0) ? 60 : -60);

      while(ABS(ReadRight() - rInit) < ABS(tVal) || ABS(ReadLeft() - lInit) < ABS(tVal)){ }

      Stop();
  }
  void Drivetrain::ApplyPower(int lPow, int rPow){
      curPowerL = Lerp(curPowerL, lPow, 0.1f);
      curPowerR = Lerp(curPowerR, rPow, 0.1f);
      leftMotors.spin(vex::reverse, curPowerL, vex::rpm);
      rightMotors.spin(vex::forward, curPowerR, vex::rpm);
  }
  void Drivetrain::Spin(float revs){
    ResetPositions();
    rightMotors.spinTo(-revs, vex::rotationUnits::rev, false);
    leftMotors.spinTo(revs, vex::rotationUnits::rev, true);
  }

  void Drivetrain::DriveDist(float dL, float dR, int sec){
      float 
          lSpeed = dL / sec / 60 / wheelCirc, 
          rSpeed = dR / sec / 60 / wheelCirc;

      SpinR(rSpeed);
      SpinL(lSpeed);

      while(ReadRight() <= rSpeed || ReadLeft() <= lSpeed) { }
  }
  void Drivetrain::Stop(){
      curPowerL = 0;
      curPowerR = 0;

      rightMotors.stop(vex::brakeType::brake);

      leftMotors.stop(vex::brakeType::brake);
  }
  float Drivetrain::ReadLeft(){
    return leftMotors.position(vex::degrees);
  }
  float Drivetrain::ReadRight(){
    return rightMotors.position(vex::degrees);
  }

  void Drivetrain::SetSpeed(float speed, vex::percentUnits units){
    rightMotors.setVelocity(speed, units);
    leftMotors.setVelocity(speed, units);
  }

  void Drivetrain::ResetPositions(){
    leftMotors.resetPosition();
    rightMotors.resetPosition();
  }

  void Drivetrain::SpinR(float revs){
    rightMotors.spin(vex::forward, revs, vex::rpm);
  }
  void Drivetrain::SpinL(float revs){
    leftMotors.spin(vex::reverse, revs, vex::rpm);
  }
//

// Drivetrain6
void Drivetrain6::ApplyPower(int lPow, int rPow){
    curPowerL = MCEC::Lerp(curPowerL, lPow, 0.05f);
    curPowerR = MCEC::Lerp(curPowerR, rPow, 0.025f);

    _mR1.spin(vex::forward, curPowerR, vex::rpm);
    _mR2.spin(vex::forward, curPowerR, vex::rpm);
    _mR3.spin(vex::forward, curPowerR, vex::rpm);

    _mL1.spin(vex::reverse, curPowerL, vex::rpm);
    _mL2.spin(vex::reverse, curPowerL, vex::rpm);
    _mL3.spin(vex::reverse, curPowerL, vex::rpm);
}

void Drivetrain6::Reset(){
  _mR1.resetPosition();
  _mR2.resetPosition();
  _mR3.resetPosition();

  _mL1.resetPosition();
  _mL2.resetPosition();
  _mL3.resetPosition();
}

void Drivetrain6::Stop(vex::brakeType br = vex::brakeType::brake){
    curPowerL = 0;
    curPowerR = 0;

    _mR1.stop(br);
    _mR2.stop(br);
    _mR3.stop(br);

    _mL1.stop(br);
    _mL2.stop(br);
    _mL3.stop(br);
}


void Controller::Set(){
    rStick.Set(
        controller.Axis1.position(), 
        controller.Axis2.position()
    );
    lStick.Set(
        controller.Axis4.position(), 
        controller.Axis3.position()
    );

    R1.Set(controller.ButtonR1.pressing());
    L1.Set(controller.ButtonL1.pressing());
    R2.Set(controller.ButtonR2.pressing());
    L2.Set(controller.ButtonL2.pressing());

    X.Set(controller.ButtonX.pressing());
    Y.Set(controller.ButtonY.pressing());
    B.Set(controller.ButtonB.pressing());
    A.Set(controller.ButtonA.pressing());

    Left.Set(controller.ButtonLeft.pressing());
    Right.Set(controller.ButtonRight.pressing());
    Down.Set(controller.ButtonDown.pressing());
    Up.Set(controller.ButtonUp.pressing());
}


void Button::Set(bool newValue){
    if(newValue != _value){
        if(_onPress && newValue){
            _onPress();
        }else if(_onRelease && !newValue){
            _onRelease();
        }
    }
    _value = newValue;
}

}
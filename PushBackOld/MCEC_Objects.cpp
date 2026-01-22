#include "MCEC_Objects.h"
#include "Vex.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}
namespace MCEC{

void Drivetrain8::SetInertial(vex::inertial* _inertial){
  inertial = _inertial;
}

int Drivetrain8::ReadLeft(){
    return (
        _mL1.position(vex::degrees) + 
        _mL2.position(vex::degrees) + 
        _mL3.position(vex::degrees) + 
        _mL4.position(vex::degrees)
    ) / 4;
}

int Drivetrain8::ReadRight(){
    return (
        _mL1.position(vex::degrees) + 
        _mL2.position(vex::degrees) + 
        _mL3.position(vex::degrees) + 
        _mL4.position(vex::degrees)
    ) / 4;
}

void Drivetrain8::UpdateHeading(){
    static const float wheelDist = (14.5f + 9) / 2;
    static const float wheelCirc = 2 * 1.625f * M_PI;
    static float headingMotor = 0;
    static int16_t lastR, lastL;
    int16_t curR, curL;

    curR = ReadLeft();
    curR = ReadRight();

    headingMotor += (wheelCirc * (curR - curL) / wheelDist);
    
    _heading += inertial->heading() * 0.8f + headingMotor * 0.2f;

    lastR = curR;
    lastL = curL;
}

// @param joyX: input from -100 to 100
// @param joyY: input from -100 to 100
void Drivetrain8::Drive(int joyX, int joyY){
    // Ensure the joystick inputs are within the valid range
    if(abs(joyX) > 100 || abs(joyY) > 100){
        return;
    }

    // Apply a deadzone to the joystick inputs
    #define DEADZONE 5
    joyX = (abs(joyX) < DEADZONE) ? 0 : joyX;
    joyY = (abs(joyY) < DEADZONE) ? 0 : joyY;

    // Setup the influence of each joystick axis
    float totalPower = abs(joyX) + abs(joyY);
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

void Drivetrain8::ApplyPower(int lPow, int rPow){
    curPowerL = Lerp(curPowerL, lPow, 0.1f);
    curPowerR = Lerp(curPowerR, rPow, 0.1f);

    _mR1.spin(vex::forward, curPowerR, vex::rpm);
    _mR2.spin(vex::forward, curPowerR, vex::rpm);
    _mR3.spin(vex::forward, curPowerR, vex::rpm);
    _mR4.spin(vex::forward, curPowerR, vex::rpm);

    _mL1.spin(vex::reverse, curPowerL, vex::rpm);
    _mL2.spin(vex::reverse, curPowerL, vex::rpm);
    _mL3.spin(vex::reverse, curPowerL, vex::rpm);
    _mL4.spin(vex::reverse, curPowerL, vex::rpm);
}


void Drivetrain8::Stop(){
    curPowerL = 0;
    curPowerR = 0;

    _mR1.stop(vex::brakeType::brake);
    _mR2.stop(vex::brakeType::brake);
    _mR3.stop(vex::brakeType::brake);
    _mR4.stop(vex::brakeType::brake);

    _mL1.stop(vex::brakeType::brake);
    _mL2.stop(vex::brakeType::brake);
    _mL3.stop(vex::brakeType::brake);
    _mL4.stop(vex::brakeType::brake);
}

void Drivetrain8::Rotate(int deg){
  while(inertial->heading() < deg){
    Drive(0, 30);
  }
}

void Drivetrain8::Spin(float revs){
  _mR1.spinTo(revs, vex::rotationUnits::rev, false);
  _mR2.spinTo(revs, vex::rotationUnits::rev, false);
  _mR3.spinTo(revs, vex::rotationUnits::rev, false);
  _mR4.spinTo(revs, vex::rotationUnits::rev, false);

  _mL1.spinTo(revs, vex::rotationUnits::rev, false);
  _mL2.spinTo(revs, vex::rotationUnits::rev, false);
  _mL3.spinTo(revs, vex::rotationUnits::rev, false);
  _mL4.spinTo(revs, vex::rotationUnits::rev, true);
}

void Drivetrain8::DriveDist(int dL, int dR){

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
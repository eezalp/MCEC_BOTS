#include "MCEC_Objects.h"
#include "Vex.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}
namespace MCEC{

void Drivetrain6::Drive(int joyX, int joyY){
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

    int Lpower = (xPower + yPower);
    int Rpower = (xPower - yPower);

    // Apply the power to the motors
    ApplyPower(Lpower, Rpower);
}

void Drivetrain6::ApplyPower(int lPow, int rPow){
    curPowerL = Lerp(curPowerL, lPow, 0.05f);
    curPowerR = Lerp(curPowerR, rPow, 0.025f);

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
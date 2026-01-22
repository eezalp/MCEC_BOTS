#include "MCEC_Objects.h"

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}
namespace MCEC{

void Drivetrain8::SetInertial(vex::inertial* _inertial){
  inertial = _inertial;
}


void Drivetrain8::UpdateHeading(){
    static float headingMotor = 0;
    static int16_t lastR, lastL;
    int16_t curR, curL;

    curR = ReadLeft();
    curR = ReadRight();

    headingMotor += (wheelCirc * ((curR - lastR) - (curL - lastL)) / wheelDist);
    
    _heading = inertial->heading();// * 0.8f + headingMotor * 0.2f;


    lastR = curR;
    lastL = curL;
}


void Drivetrain8::Rotate(int deg, bool cw){
    float initialHeading = _heading;
    float target = (deg + initialHeading > 360) ? (deg + initialHeading - 360) : (deg + initialHeading < 0) ? (deg + initialHeading + 360) : (deg + initialHeading);
  
    // Brain.Screen.setCursor(7, 1);
    // Brain.Screen.print("%f", initialHeading);
    // Brain.Screen.print("       ");
    // Brain.Screen.setCursor(9, 1);
    // Brain.Screen.print("%f", target);
    // Brain.Screen.print("       ");

    if(_heading > target){
        // Brain.Screen.setCursor(8, 1);
        // Brain.Screen.print("Greater");
        Drive(0, 100);
        while(_heading > target){
            // Brain.Screen.setCursor(8, 1);
            // Brain.Screen.print("%f", _heading);
            // Brain.Screen.print("       ");
            UpdateHeading();
        }
    }else if(_heading < target) {
        // Brain.Screen.setCursor(8, 1);
        // Brain.Screen.print("Less");
        Drive(0, -100);
        while(_heading < target){
            // Brain.Screen.setCursor(8, 1);
            // Brain.Screen.print("%f", _heading);
            // Brain.Screen.print("       ");
            UpdateHeading();
        }
    }

}


void Drivetrain8::Rotate2(float targ){
    float tVal = (targ / 2) * wheelDist;
    float rInit = ReadRight(), lInit = ReadLeft();

    SpinR((targ < 0) ? -60 : 60);
    SpinL((targ < 0) ? 60 : -60);

    // Brain.Screen.setCursor(4, 1);
    // Brain.Screen.print("%f", rInit);
    // Brain.Screen.print("       ");
    // Brain.Screen.setCursor(5, 1);
    // Brain.Screen.print("%f", lInit);
    // Brain.Screen.print("       ");
    // Brain.Screen.setCursor(8, 1);
    // Brain.Screen.print("%f", tVal);
    // Brain.Screen.print("       ");

    while(abs(ReadRight() - rInit) < abs(tVal) || abs(ReadLeft() - lInit) < abs(tVal)){
        // Brain.Screen.setCursor(6, 1);
        // Brain.Screen.print("%f", ReadRight() - rInit);
        // Brain.Screen.print("       ");
        // Brain.Screen.setCursor(7, 1);
        // Brain.Screen.print("%f", ReadLeft() - lInit);
        // Brain.Screen.print("       ");
    }
    // Brain.Screen.setCursor(6, 1);
    // Brain.Screen.print("%f", ReadRight() - rInit);
    // Brain.Screen.print("       ");
    // Brain.Screen.setCursor(7, 1);
    // Brain.Screen.print("%f", ReadLeft() - lInit);
    // Brain.Screen.print("       ");
    // Brain.Screen.setCursor(8, 1);
    // Brain.Screen.print("%f", tVal);
    // Brain.Screen.print("       ");

    Stop();
}

float Drivetrain8::ReadLeft(){
    return (
        _mL1.position(vex::degrees) + 
        _mL2.position(vex::degrees) + 
        _mL3.position(vex::degrees) + 
        _mL4.position(vex::degrees)
    ) / 4;
}

float Drivetrain8::ReadRight(){
    return (
        _mL1.position(vex::degrees) + 
        _mL2.position(vex::degrees) + 
        _mL3.position(vex::degrees) + 
        _mL4.position(vex::degrees)
    ) / 4;
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

void Drivetrain8::Spin(float revs){
  _mR1.resetPosition();
  _mR2.resetPosition();
  _mR3.resetPosition();
  _mR4.resetPosition();
  _mL1.resetPosition();
  _mL2.resetPosition();
  _mL3.resetPosition();
  _mL4.resetPosition();

  _mR1.spinTo(-revs, vex::rotationUnits::rev, false);
  _mR2.spinTo(-revs, vex::rotationUnits::rev, false);
  _mR3.spinTo(-revs, vex::rotationUnits::rev, false);
  _mR4.spinTo(-revs, vex::rotationUnits::rev, false);

  _mL1.spinTo(revs, vex::rotationUnits::rev, false);
  _mL2.spinTo(revs, vex::rotationUnits::rev, false);
  _mL3.spinTo(revs, vex::rotationUnits::rev, false);
  _mL4.spinTo(revs, vex::rotationUnits::rev, true);
}

void Drivetrain8::SpinR(float revs){
  _mR1.spin(vex::forward, revs, vex::rpm);
  _mR2.spin(vex::forward, revs, vex::rpm);
  _mR3.spin(vex::forward, revs, vex::rpm);
  _mR4.spin(vex::forward, revs, vex::rpm);
}
void Drivetrain8::SpinL(float revs){
  _mL1.spin(vex::reverse, revs, vex::rpm);
  _mL2.spin(vex::reverse, revs, vex::rpm);
  _mL3.spin(vex::reverse, revs, vex::rpm);
  _mL4.spin(vex::reverse, revs, vex::rpm);
}

void Drivetrain8::SetSpeed(float speed, vex::percentUnits units){
    _mR1.setVelocity(speed, units);
    _mR2.setVelocity(speed, units);
    _mR3.setVelocity(speed, units);
    _mR4.setVelocity(speed, units);

    _mL1.setVelocity(speed, units);
    _mL2.setVelocity(speed, units);
    _mL3.setVelocity(speed, units);
    _mL4.setVelocity(speed, units);
}

void Drivetrain8::DriveDist(float dL, float dR, int sec){
    float 
        lSpeed = dL / sec / 60 / wheelCirc, 
        rSpeed = dR / sec / 60 / wheelCirc;
    int rStart = ReadRight();
    int lStart = ReadLeft();

    SpinR(rSpeed);
    SpinL(lSpeed);

    while(ReadRight() <= rSpeed || ReadLeft() <= lSpeed) { }
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
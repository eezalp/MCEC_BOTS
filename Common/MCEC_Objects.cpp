#include "MCEC_Objects.h"
#include <algorithm>

float MCEC::Lerp(float a, float b, float t){
  return ((1 - t) * a) + (b * t);
}

template <typename T>
T inline Clamp2(T n, T min, T max){
  return (n < min) ? min : ((n > max) ? max : n);
}

namespace MCEC{
  
float WrapAngle(float angle) {
  while (angle > 180) angle -= 360;   // 270° becomes -90°
  while (angle < -180) angle += 360;  // -200° becomes 160°
  return angle;
}

float abs2(float n){
  if(n < 0) return n * -1;
  return n;
}

float AngleDiff(float a_1, float a_2){
  float a1 = a_1 * (M_PI / 180.0f);
  float a2 = a_2 * (M_PI / 180.0f);
  // return ((a_2 - a_1) - 180) % 360 + 180;
  return std::atan2(std::sin(a1 - a2), std::cos(a1 - a2)) * (180.0f/M_PI);
}

bool Joystick::isMoved(){
  return (abs2(x) >= 1.0f || abs2(y) >= 1.0f);
}

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
    Rpower = (xPower - yPower);    // balls

    // Apply the power to the motors
    ApplyPower(Lpower * leftMulti, Rpower * rightMulti);
  }

  void Drivetrain::Rotate(float targ){
    uint32_t startTime = Brain.Timer.system();
    uint32_t lastTime = startTime;
    uint32_t curTime;
    float power = 0;
    float dt = 0;

    pid.Prime(_heading, targ);
    dt = (Brain.Timer.system() - lastTime) / 1000.0f;

    while(!pid.AtTarget(_heading, dt)){
      curTime = Brain.Timer.system();
      dt = (curTime - lastTime) / 1000.0f;

      UpdateHeading();
      power = pid.Update(_heading, dt);

      SpinR(-power);
      SpinL(power);

      lastTime = curTime;
    }

    Stop();
  }

  void Drivetrain::Rotate(float targ, float power){
      float tVal = (targ * (M_PI/180.0f)) * (3.0f/5.0f) * wheelDistance / wheelCirc;
      float rInit = ReadRight(), lInit = ReadLeft();
      float inertialInitial = inertial->heading();

      SpinR((targ < 0) ? -power : power);
      SpinL((targ < 0) ? power : -power);

      while(
        (abs2(ReadRight() - rInit) < abs2(tVal) || abs2(ReadLeft() - lInit) < abs2(tVal)) &&
        (AngleDiff(inertialInitial, inertial->heading()) < abs2(targ))
    ){
      }

      Stop();
      inertial->setHeading(0, vex::degrees);
  }

  void Drivetrain::Spin(float revs, float power){
    float rInit = ReadRight(), lInit = ReadLeft();
    float curPow = power;
    
    while(abs2(ReadRight() - rInit) < abs2(revs) || abs2(ReadLeft() - lInit) < abs2(revs)){
      SpinR((revs < 0) ? curPow : -curPow);
      SpinL((revs < 0) ? curPow : -curPow);
    }
  }

  void Drivetrain::UpdateHeading(){
      static float lastR = ReadRight(), lastL = ReadLeft();
      static float prevHeading = inertial->heading();
      float dMotor = 0;
      float curR = ReadRight(), curL = ReadLeft();
      float curHeading = inertial->heading();
      float dHeading = AngleDiff(curHeading, prevHeading);

      dMotor += (wheelCirc * ((curR - lastR) - (curL - lastL))) / wheelDist;
      
      _heading += dHeading * 1.0f + dMotor * 0.0f;
      _heading = curHeading;

      lastR = curR;
      lastL = curL;
      prevHeading = curHeading;
  }

  void Drivetrain::SetInertial(vex::inertial* _inertial){
    inertial = _inertial;
  }

  void Drivetrain::ApplyPower(int lPow, int rPow){
    curPowerL = (abs2(curPowerL) > abs2(lPow)) ? Lerp(curPowerL, lPow, 0.075f) : lPow;
    curPowerR = (abs2(curPowerR) > abs2(rPow)) ? Lerp(curPowerR, rPow, 0.075f) : rPow;
    leftMotors.Spin(vex::reverse, curPowerL, vex::rpm);
    rightMotors.Spin(vex::forward, curPowerR, vex::rpm);
  }

  void Drivetrain::DriveDist(float d, float sec){
    float dd = d / wheelCirc;
    float 
      lSpeed = d / sec / 60 / wheelCirc, 
      rSpeed = d / sec / 60 / wheelCirc;

    SpinR(rSpeed);
    SpinL(lSpeed);

    while(ABS(ReadRight()) <= ABS(dd) || ABS(ReadLeft()) <= ABS(dd)) { }
  }

  void Drivetrain::DriveDist(float dL, float dR, float sec){
    float 
      lSpeed = dL / sec / 60 / wheelCirc, 
      rSpeed = dR / sec / 60 / wheelCirc;

    SpinR(rSpeed);
    SpinL(lSpeed);

    while(ReadRight() <= rSpeed || ReadLeft() <= lSpeed) { }
  }
  void Drivetrain::Stop(vex::brakeType br){
    curPowerL = 0;
    curPowerR = 0;

    rightMotors.Stop(br);
    leftMotors.Stop(br);
  }
  float Drivetrain::ReadLeft(){
    return leftMotors.Position(vex::rotationUnits::rev);
  }
  float Drivetrain::ReadRight(){
    return rightMotors.Position(vex::rotationUnits::rev);
  }

  void Drivetrain::SetSpeed(float speed, vex::percentUnits units){
    leftMotors.SetVelocity(speed, units);
    rightMotors.SetVelocity(speed, units);
  }

  void Drivetrain::ResetPositions(){
    leftMotors.ResetPosition();
    rightMotors.ResetPosition();
  }

  void Drivetrain::SpinR(float revs){
    rightMotors.Spin(vex::forward, revs, vex::rpm);
  }
  void Drivetrain::SpinL(float revs){
    leftMotors.Spin(vex::reverse, revs, vex::rpm);
  }
//
// Rotation PID
  void RotationPID::SetVariables(float P, float I, float D){
    m_P = P;
    m_I = I;
    m_D = D;
  }

  void RotationPID::Reset(){
    _integral = 0;
    _prevError = 0;
  }

  void RotationPID::Prime(float curAngle, float targAngle){
    _prevError = AngleDiff(targAngle, curAngle);
    _target = targAngle;
  }

  bool RotationPID::AtTarget(float curAngle, float dt){
    static bool lastYes = false;
    if(abs2(curAngle - _target) <= 0.5f){
        if(lastYes){
            return true;
        }
        lastYes = true;
    }else{
        lastYes = false;
    }
    return false;
  }

  float RotationPID::Update(float error, float dt){
    _integral += error * dt;
    _integral = Clamp(_integral, -_integralMax, _integralMax);
    
    float derivative = (error - _prevError) / dt;
    
    //PPID
    float P = m_P * error;
    float I = m_I * _integral;
    float D = m_D * derivative;
    
    _prevError = error;
    
    float output = P + I + D;
    
    return Clamp2(output, -_maxOutput, _maxOutput);
  }

  void RotationPID::SetTarget(float targ){
    _target = targ;
  }
//

// PID
  void PID::SetVariables(float P, float I, float D){
    m_P = P;
    m_I = I;
    m_D = D;
  }

  void PID::Reset(){
    _integral = 0;
    _prevError = 0;
  }

  void PID::Prime(float cur, float targ){
    _prevError = targ - cur;
    _target = targ;
  } 

  float PID::Update(float error, float dt){
    
    _integral += error * dt;
    _integral = Clamp(_integral, -_integralMax, _integralMax);
    
    float derivative = (error - _prevError) / dt;
    
    //PPID
    float P = m_P * error;
    float I = m_I * _integral;
    float D = m_D * derivative;

    
    _prevError = error;
    
    float output = P + I + D;
    
    return Clamp2(output, -_maxOutput, _maxOutput);
  }

  void PID::SetTarget(float targ){
    _target = targ;
  }
//

void Controller::Set(){
    rStick.Set(
        JoystickCurve(controller.Axis1.position() / 100.0f) * 100.0f, 
        JoystickCurve(controller.Axis2.position() / 100.0f) * 100.0f
    );
    lStick.Set(
        JoystickCurve(controller.Axis4.position() / 100.0f) * 100.0f, 
        JoystickCurve(controller.Axis3.position() / 100.0f) * 100.0f
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
float Controller::JoystickCurve(float input){
  // preserves sign, applies curve to magnitude
  return std::copysign(std::pow(std::abs(input), exponent), input);
}

std::vector<PathPoint> pathFromFile(const char* filename) {
  std::vector<PathPoint> points;

  int32_t size = Brain.SDcard.size(filename);
  if (size <= 0) return points;

  uint8_t* buf = new uint8_t[size + 1];
  Brain.SDcard.loadfile(filename, buf, size);
  buf[size] = '\0';

  char* line = strtok((char*)buf, "\n");
  while (line != nullptr) {
      PathPoint p = {{0, 0}, 0, 0};
      sscanf(line, "%f,%f,%f,%f", &p.point.x, &p.point.y, &p.heading, &p.speed);
      points.push_back(p);
      line = strtok(nullptr, "\n");
  }

  delete[] buf;
  return points;
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
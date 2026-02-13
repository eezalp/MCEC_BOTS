#include "MCEC_Objects.h"
#include <algorithm>

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
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
        // controls.controller.Screen.clearScreen();
        // controls.controller.Screen.setCursor(1, 13);
        // controls.controller.Screen.print(inertialInitial);
        // controls.controller.Screen.setCursor(2, 13);
        // controls.controller.Screen.print(abs2(inertialInitial - inertial->heading()));
        // controls.controller.Screen.setCursor(3, 13);
        // controls.controller.Screen.print(abs2(targ));

      while(
        (abs2(ReadRight() - rInit) < abs2(tVal) || abs2(ReadLeft() - lInit) < abs2(tVal)) &&
        (AngleDiff(inertialInitial, inertial->heading()) < abs2(targ))
    ){
        // controls.controller.Screen.setCursor(1, 7);
        // controls.controller.Screen.print(ReadRight());
        // controls.controller.Screen.setCursor(2, 7);
        // controls.controller.Screen.print(ReadLeft());
        // controls.controller.Screen.setCursor(2, 13);
        // controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
      }

        // controls.controller.Screen.setCursor(2, 13);
        // controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
      Stop();
      inertial->setHeading(0, vex::degrees);
        // controls.controller.Screen.setCursor(1, 7);
        // controls.controller.Screen.print(AngleDiff(inertialInitial, inertial->heading()));
        // controls.controller.Screen.setCursor(2, 7);
        // controls.controller.Screen.print(ReadLeft());
  }
  void Drivetrain::Spin(float revs, float power){
    float rInit = ReadRight(), lInit = ReadLeft();
    float curPow = power;
    // leftMotors.SpinTo(revs, vex::rotationUnits::rev, false);
    // rightMotors.SpinTo(revs, vex::rotationUnits::rev, true);
      while(abs2(ReadRight() - rInit) < abs2(revs) || abs2(ReadLeft() - lInit) < abs2(revs)){
        // curPow = Lerp(curPow, power, 0.075f);
        SpinR((revs < 0) ? curPow : -curPow);
        SpinL((revs < 0) ? curPow : -curPow);
        // vex::this_thread::sleep_for(10);
      }

    //   Stop();

    // controls.controller.Screen.setCursor(1, 1);
    // controls.controller.Screen.print(ReadRight());
    // controls.controller.Screen.setCursor(2, 1);
    // controls.controller.Screen.print(ReadLeft());
    // controls.controller.Screen.setCursor(3, 7);
    // controls.controller.Screen.print(revs);
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
    // controls.controller.Screen.setCursor(2, 1);
    // controls.controller.Screen.print("Heading: %.2f    ", _heading);

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

  float RotationPID::Update(float curAngle, float dt){
    float error = AngleDiff(_target, curAngle);
    float derivative = (error - _prevError) / dt;
    float P = 0, I = 0, D = 0;
    _integral += error * dt;

    P = m_P * error;
    I = m_I * _integral;
    D = m_D * derivative;

    _prevError = error;

    return P + I + D;
  }

  void RotationPID::SetTarget(float targ){
    _target = targ;
  }
//
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

// SwervePod
  void SwervePod::SetPowers(Vector2 power){
    float currentPodAngle = rotation.angle(vex::degrees);
    
    float rotationPower = GetPivot(power, currentPodAngle);
    
    float driveSpeed = power.GetMagnitude();
    
    if (reversed) {
        driveSpeed = -driveSpeed;
    }

    float topPower = driveSpeed + (rotationPower * MAX_ROTATION_POWER);
    float botPower = driveSpeed - (rotationPower * MAX_ROTATION_POWER);
    
    float maxPower = std::max(std::abs(topPower), std::abs(botPower));
    if (maxPower > MAX_MOTOR_POWER) {
        topPower *= MAX_MOTOR_POWER / maxPower;
        botPower *= MAX_MOTOR_POWER / maxPower;
    }
    
    top.spin(vex::forward, topPower, vex::pct);
    bottom.spin(vex::forward, botPower, vex::pct);
  }
  void SwervePod::GoToVector(Vector2 targ){
    float move = targ.GetMagnitude();
    float pivot = GetPivot(targ, rotation.angle(vex::degrees));
    Vector2 powerVector(move, pivot);
    if(abs2(move) > 0){
        SetPowers(powerVector);
    }
  }
  float SwervePod::GetPivot(Vector2 target, float angle){
    float angleDiff = AngleDiff(target.GetAngle(), angle);
    
    if(std::abs(angleDiff) > 110){ 
        if(!onShortest){
            reversed = !reversed;
        }
        onShortest = true;
    }else{
        onShortest = false;
    }

    if(std::abs(angleDiff) < ROTATION_ERROR){
        return 0;
    }else if(std::abs(angleDiff) > MAX_ROTATION_POWER){
        return (angleDiff > 0) ? 1.0f : -1.0f; 
    }else{
        return angleDiff / MAX_ROTATION_POWER;  
    }
  }
  void SwervePod::Brake(vex::brakeType br){
    top.stop(br);
    bottom.stop(br);
  }
  
  const Vector2 SwervePod::MOTOR_TOP_VECTOR = Vector2(1/std::sqrt(2), 1/std::sqrt(2));
  const Vector2 SwervePod::MOTOR_BOT_VECTOR = Vector2(-1/std::sqrt(2), 1/std::sqrt(2));
//

// SwerveDrive
  void SwerveDrive::Drive(Vector2 driveVector, float rotationSpeed){
    Vector2 FL_pos(-trackwidth/2,  wheelbase/2);
    Vector2 FR_pos( trackwidth/2,  wheelbase/2);
    Vector2 BL_pos(-trackwidth/2, -wheelbase/2);
    Vector2 BR_pos( trackwidth/2, -wheelbase/2);
    
    // Rotation creates tangential velocity perpendicular to radius
    // For CCW rotation: tangent = (-y, x) * rotationSpeed
    Vector2 FL_rot(-FL_pos.y * rotationSpeed, FL_pos.x * rotationSpeed);
    Vector2 FR_rot(-FR_pos.y * rotationSpeed, FR_pos.x * rotationSpeed);
    Vector2 BL_rot(-BL_pos.y * rotationSpeed, BL_pos.x * rotationSpeed);
    Vector2 BR_rot(-BR_pos.y * rotationSpeed, BR_pos.x * rotationSpeed);
    
    // Combine translation + rotation for each pod
    Vector2 FL_target = driveVector + FL_rot;
    Vector2 FR_target = driveVector + FR_rot;
    Vector2 BL_target = driveVector + BL_rot;
    Vector2 BR_target = driveVector + BR_rot;
    
    // Normalize if any pod exceeds max speed
    float maxMag = std::max({FL_target.GetMagnitude(), 
                             FR_target.GetMagnitude(),
                             BL_target.GetMagnitude(), 
                             BR_target.GetMagnitude()});
    
    if (maxMag > MAX_SPEED) {
        float scale = MAX_SPEED / maxMag;
        FL_target *= scale;
        FR_target *= scale;
        BL_target *= scale;
        BR_target *= scale;
    }
    
    // Send to each pod
    frontLeft.GoToVector(FL_target);
    frontRight.GoToVector(FR_target);
    backLeft.GoToVector(BL_target);
    backRight.GoToVector(BR_target);
  }
  void SwerveDrive::Stop(vex::brakeType br){
    frontLeft.Brake(br); frontRight.Brake(br);
    backLeft.Brake(br);  backRight.Brake(br);
  }
//

}
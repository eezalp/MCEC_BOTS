#include "MCEC_Objects.h"
#include <algorithm>

float MCEC::Lerp(float a, float b, float t){
    return ((1 - t) * a) + (b * t);
}
template <typename T>
T inline max2(T a, T b){
  return a > b ? a : b;
}

template <typename T>
T inline Clamp2(T n, T min, T max){
  return (n < min) ? min : ((n > max) ? max : n);
}

namespace MCEC{

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
  return (abs2(x) >= 10.0f || abs2(y) >= 10.0f);
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

float WrapAngle(float angle) {
  while (angle > 180) angle -= 360;   // 270° becomes -90°
  while (angle < -180) angle += 360;  // -200° becomes 160°
  return angle;
}

// SwervePod
  void SwervePod::SetPowers(float move, float rot, bool straight){
    if(abs2(rot) < 0.1f) rot = 0;
    float topPower = move - (rot * 100.0f);
    float botPower = move + (rot * 100.0f);

    // if(screen < 4){
    //   controls.controller.Screen.setCursor(2, 1);
    //   controls.controller.Screen.print("%.2f, %.2f", topPower, botPower);
    // }
    // if(screen < 4){
    //   controls.controller.Screen.setCursor(3, 1);
    //   controls.controller.Screen.print("%.2f, %.2f", move, rot);
    // }

    float maxPower = max2(abs2(topPower), abs2(botPower));
    if (maxPower > MAX_MOTOR_POWER) {
      topPower *= MAX_MOTOR_POWER / maxPower;
      botPower *= MAX_MOTOR_POWER / maxPower;
    }


    if(abs2(topPower) < 0.01f){
      top.stop(vex::brakeType::hold);
    }else{
      top.spin(reversed ? vex::reverse : vex::forward, topPower, vex::pct);
    }
    if(abs2(botPower) < 0.01f){
      bottom.stop(vex::brakeType::hold);
    }else{
      bottom.spin(reversed ? vex::forward : vex::reverse, botPower, vex::pct);
    }

    // top.spin(vex::forward, topPower, vex::pct);
    // bottom.spin(vex::reverse, botPower, vex::pct);

  }

  void SwervePod::GoToVector(Vector2 targ, bool canForward, float dt){
    static const int deadzoneAngle = 95;
    
    float angleDiffForward = 0;
    float curAngle = GetAngle();
    float move = targ.GetMagnitude();
    float angl = targ.GetAngle();

    angleDiffForward = (AngleDiff(WrapAngle(angl), WrapAngle(curAngle)));
      
    if (angleDiffForward > deadzoneAngle || angleDiffForward < -deadzoneAngle)
      reversed = true;
    else if(angleDiffForward < deadzoneAngle || angleDiffForward > -deadzoneAngle)
      reversed = false;
    
    float angleDiff = AngleDiff(WrapAngle(angl + (reversed ? 180 : 0)), WrapAngle(curAngle));

    // .2 for less wiggily and 2 for better driving
    float angleDampen = std::sqrt(abs2(angleDiff/90.0f) * (master->runningAuton ? 0.2f : 2.0f));
    float rot = rotationPID.Update(angleDiff, dt) * angleDampen * (reversed ? -1 : 1);
    float denom = std::fmax(std::sqrt(abs2(angleDiff/90.0f) * 2), 1.0f);

    Vector2 powerVector((canForward ? move : 0) / denom, rot);
    move = (canForward ? move : 0) / denom;
    
    volatile float topPower = move - (rot * 100.0f);
    volatile float botPower = move + (rot * 100.0f);
    if(screen < 4){
      controls.controller.Screen.setCursor(2, 1);
      controls.controller.Screen.print("%.2f, %.2f", topPower, move - (rot * 100.0f));
      controls.controller.Screen.setCursor(3, 1);
      controls.controller.Screen.print("%.2f, %.2f", move, rot);
    }
    float maxPower = max2(abs2(topPower), abs2(botPower));
    if (maxPower > MAX_MOTOR_POWER) {
      topPower *= MAX_MOTOR_POWER / maxPower;
      botPower *= MAX_MOTOR_POWER / maxPower;
    }

    if(abs2(topPower) < 0.01f){
      top.stop(vex::brakeType::hold);
    }else{
      top.spin(reversed ? vex::reverse : vex::forward, topPower, vex::pct);
    }
    if(abs2(botPower) < 0.01f){
      bottom.stop(vex::brakeType::hold);
    }else{
      bottom.spin(reversed ? vex::forward : vex::reverse, botPower, vex::pct);
    }

  }


  void SwervePod::SetPIDVariables(float P, float I, float D){
    rotationPID.SetVariables(P, I, D);
  }

  void SwervePod::Brake(vex::brakeType br){
    top.stop(br);
    bottom.stop(br);
  }

  float SwervePod::GetAngle(){
    float angle = rotation.angle() + rotationOffset;// + (reversed ? 180 : 0);
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
    return angle;
  }

  bool SwervePod::IsMoving(){
    return (abs2(top.velocity(vex::velocityUnits::pct)) <= 0.01f && abs2(bottom.velocity(vex::velocityUnits::pct)) <= 0.01f);
  }

  void SwervePod::SetRotationOffset(float _off){
    rotationOffset = _off;
  }

  float SwervePod::CalcWub(float g){
    static float topOld = top.position(vex::rotationUnits::deg);
    static float botOld = bottom.position(vex::rotationUnits::deg);
    float heading = GetAngle() * (M_PI/180.0f);

    float topCur = top.position(vex::rotationUnits::deg);
    float botCur = bottom.position(vex::rotationUnits::deg);

    delTop = AngleDiff(topCur, topOld) * (M_PI / 180.0f);
    delBot = AngleDiff(botCur, botOld) * (M_PI / 180.0f);

    float wheelWub = ((delTop + delBot) / 2);
    float tangential = (wheelWub * std::sin(heading) * relPos.x) - (wheelWub * std::cos(heading) * relPos.y);
    float wub = tangential / (relPos.GetMagnitude() * relPos.GetMagnitude());

    topOld = topCur;
    botOld = botCur;

    return wub;
  }
  Vector2 SwervePod::GetTraveled(float dt, float wub) {
    float heading = GetAngle() * (M_PI/180.0f);
    float topCur = top.position(vex::rotationUnits::deg);
    float botCur = bottom.position(vex::rotationUnits::deg);
    
    float wheelWub = ((delTop + delBot) / 2);
    
    Vector2 relativeVelocity(
      (std::cos(heading) * wheelWub) - (wub * relPos.y),
      (std::sin(heading) * wheelWub) + (wub * relPos.x)
    );

    // return relativeVelocity;

    return Vector2(
        std::cos(heading) * wheelWub,
        std::sin(heading) * wheelWub
    );
  }
  
  const Vector2 SwervePod::MOTOR_TOP_VECTOR = Vector2(1/std::sqrt(2), 1/std::sqrt(2));
  const Vector2 SwervePod::MOTOR_BOT_VECTOR = Vector2(-1/std::sqrt(2), 1/std::sqrt(2));
///

// SwerveDrive
  void SwerveDrive::UpdatePositionUsingAccel(float dt){
    float x = inertial.acceleration(vex::axisType::xaxis);
    float y = inertial.acceleration(vex::axisType::yaxis);
    if(abs2(x) < .1f) x = 0;
    if(abs2(y) < .1f) y = 0;

    currentAccel = Vector2(
      x * 32.2f * 12, 
      y * 32.2f * 12
    );
    currentVel += currentAccel * dt;
    if(
      !frontLeft.IsMoving() && !frontRight.IsMoving() &&
       !backLeft.IsMoving() && !backRight.IsMoving()
    ){
      currentVel = Vector2(0, 0);
    }
    currentPos += currentVel * dt;
  }
  void SwerveDrive::UpdatePositionUsingEncoder(float dt){
    Vector2 total = Vector2::zero;
    float heading = inertial.heading();
    float wub = 
      (frontLeft.CalcWub(heading) + frontRight.CalcWub(heading) + 
      backLeft.CalcWub(heading) + backRight.CalcWub(heading)) / 
      4.0f;
    
    wub = inertial.gyroRate(vex::axisType::zaxis, vex::velocityUnits::dps) * (M_PI / 180.0f);
    
    total += frontLeft.GetTraveled(dt, wub);
    total += frontRight.GetTraveled(dt, wub);
    total += backLeft.GetTraveled(dt, wub);
    total += backRight.GetTraveled(dt, wub);

    total = total / 4;

    currentPos += total;

  }
  void SwerveDrive::UpdatePositionUsingOdom(float dt){
    static float lastHeading = inertial.heading();
    float heading = inertial.heading();
    float dTheta = AngleDiff(heading, lastHeading) * (M_PI/180.0f);
    Vector2 posChange = Vector2(
      forward.GetDeltaDistance() - (forward.offset.x * dTheta),
      lateral.GetDeltaDistance() - (lateral.offset.y * dTheta)
    );

    currentPos += posChange.Rotate(((heading + lastHeading)) / 2);
    lastHeading = heading;
  }
  void SwerveDrive::UpdatePosition(float dt){
    // UpdatePositionUsingEncoder(dt);
    // UpdatePositionUsingAccel(dt);
    UpdatePositionUsingOdom(dt);

    // controls.controller.Screen.setCursor(3, 1);
    // controls.controller.Screen.print("(%.2f, %.2f)                 ", currentPos.x, currentPos.y);
  }
  void SwerveDrive::Drive(Vector2 driveVector, float rotationSpeed){
    static const Vector2 FL_pos(-1,  1);
    static const Vector2 FR_pos( 1,  1);
    static const Vector2 BL_pos(-1, -1);  
    static const Vector2 BR_pos( 1, -1);
    
    Vector2 FL_rot(-FL_pos.y * rotationSpeed,  FL_pos.x * rotationSpeed);
    Vector2 FR_rot(FR_pos.y * rotationSpeed,  -FR_pos.x * rotationSpeed);
    Vector2 BL_rot(BL_pos.y * rotationSpeed,  -BL_pos.x * rotationSpeed);
    Vector2 BR_rot(-BR_pos.y * rotationSpeed,  BR_pos.x * rotationSpeed);
    
    driveVector.Rotate(-inertial.heading());

    Vector2 FL_target = driveVector + FL_rot;
    Vector2 FR_target = driveVector + FR_rot;
    Vector2 BL_target = driveVector + BL_rot;
    Vector2 BR_target = driveVector + BR_rot;

    float maxMag = std::max({
      FL_target.GetMagnitude(), 
      FR_target.GetMagnitude(),
      BL_target.GetMagnitude(), 
      BR_target.GetMagnitude()
    });
    
    if (maxMag > MAX_MODULE_OUTPUT) {
      float scale = MAX_MODULE_OUTPUT / maxMag;
      FL_target *= scale;
      FR_target *= scale;
      BL_target *= scale;
      BR_target *= scale;
    }
    
    static uint32_t lastTime = Brain.Timer.system();
    uint32_t curTime = Brain.Timer.system();
    float dt = (curTime - lastTime) / 1000.0f;
    lastTime = curTime;
    
    // controls.controller.Screen.setCursor(3, 1);
    // controls.controller.Screen.print("dt:%.3f", dt);

    frontLeft.GoToVector(FL_target, canForward, dt);
    frontRight.GoToVector(FR_target, canForward, dt);
    backLeft.GoToVector(BL_target, canForward, dt);
    backRight.GoToVector(BR_target, canForward, dt);
  }

  void SwerveDrive::Stop(vex::brakeType br){
    frontLeft.Brake(br); frontRight.Brake(br);
    backLeft.Brake(br);  backRight.Brake(br);
  }

  void SwerveDrive::AutonMove(Vector2 driveVector, float targetHeading, float dt){
    float power = autonMovePID.Update(driveVector.GetMagnitude(), dt);
    float angleDiff = AngleDiff(targetHeading, inertial.heading());
    float rotationPower = autonRotationPID.Update(angleDiff, dt);
    if(abs2(angleDiff) < 3) rotationPower = 0;
    driveVector = driveVector.Normalize();

    screenMutex.lock();
    // controls.controller.Screen.setCursor(1, 1);
    // controls.controller.Screen.print("%.2f, %.2f", rotationPower, angleDiff);
    // controls.controller.Screen.setCursor(3, 1);
    // controls.controller.Screen.print("%.2f, %.2f", rotationPower * 30, driveVector * power * -50);
    screenMutex.unlock();

    Drive(driveVector * power * 50, rotationPower * 30);
  }

  void SwerveDrive::SetRotationOffsets(float fl, float fr, float bl, float br){
    frontLeft.SetRotationOffset(fl); frontRight.SetRotationOffset(fr);
    backLeft.SetRotationOffset(bl); backRight.SetRotationOffset(br);

    frontLeft.screen  =  frontLeft.rotationPID.screen = 1;
    frontRight.screen = frontRight.rotationPID.screen = 4;
    backLeft.screen   =   backLeft.rotationPID.screen = 4;
    backRight.screen  =  backRight.rotationPID.screen = 4;

    frontLeft.rotationPID.Prime(frontLeft.GetAngle(), frontLeft.GetAngle());
    frontRight.rotationPID.Prime(frontRight.GetAngle(), frontRight.GetAngle());
    backLeft.rotationPID.Prime(backLeft.GetAngle(), backLeft.GetAngle());
    backRight.rotationPID.Prime(backRight.GetAngle(), backRight.GetAngle());

    frontLeft.relPos = Vector2(-trackwidth, wheelbase);
    frontRight.relPos = Vector2(trackwidth, wheelbase);
    backLeft.relPos = Vector2(-trackwidth, -wheelbase);
    backRight.relPos = Vector2(trackwidth, -wheelbase);

    frontLeft.motorRatio = frontRight.motorRatio = 1.0f/3.0f;
    backLeft.motorRatio = backRight.motorRatio = 1.0f/3.0f;
  }

  void SwerveDrive::FaceDirection(float direction){
    static RotationPID pid(0.04f, 0.0f, 0.004f);
    float start = (vex::timer::system());
    float lastTime = start;
    float curTime = (vex::timer::system());
    float dt = (curTime - lastTime) / 1000.0f;
    float aDiff = AngleDiff(inertial.heading(), direction);

    while(aDiff > 0.5f){
      curTime = (vex::timer::system());
      dt = (curTime - lastTime) / 1000.0f;
      lastTime = curTime;

      float power = pid.Update(aDiff, dt);

      Drive(Vector2::zero, power * 100);

      aDiff = AngleDiff(inertial.heading(), direction);
    }
  }

  void SwerveDrive::SetAutonPIDVariables(float moveP, float moveI, float moveD, float rotationP, float rotationI, float rotationD){
    autonMovePID.SetVariables(moveP, moveI, moveD);
    autonRotationPID.SetVariables(rotationP, rotationI, rotationD);
  }

  void SwerveDrive::GoToPoint(){
    static uint32_t start = vex::timer::system();
    static uint32_t lastTime = vex::timer::system();
    uint32_t curTime = vex::timer::system();
    float dt = (curTime - lastTime) / 1000.0f;
    //why is it 20?
    float angleDiff = 20;

    int count = 0;
    Vector2 direction(1000, 1000);
    currentPos = points.front().point;
    runningAuton = true;
    
    controls.controller.Screen.clearScreen();
    frontLeft.rotationPID.Reset();
    frontRight.rotationPID.Reset();
    backLeft.rotationPID.Reset();
    backRight.rotationPID.Reset();

    for(PathPoint &p : points){
      direction.x = p.point.x - currentPos.x;
      direction.y = p.point.y - currentPos.y;
      // angleDiff = 30;
      autonRotationPID.Reset();
      autonMovePID.Reset();
      while(direction.GetMagnitude() > 0.1f || angleDiff > 3){
        curTime = vex::timer::system();
        dt = (curTime - lastTime)/1000.0f;
        lastTime = curTime;
        if(direction.GetMagnitude() <= 0.1f){
          direction = Vector2::zero;
        }
        AutonMove(direction, p.heading, dt);
        UpdatePosition(dt);
        // controls.controller.Screen.setCursor(2, 1);
        // controls.controller.Screen.print("%.2f, %.2f", currentPos.x, currentPos.y);
        // if(!comp.isAutonomous()){
        //   return;
        // }
        direction.x = p.point.x - currentPos.x;
        direction.y = p.point.y - currentPos.y;
        angleDiff = abs2(AngleDiff(p.heading, inertial.heading()));
        vex::this_thread::sleep_for(20);
      }

      Stop();
    }
    // controls.controller.Screen.clearScreen();
    // controls.controller.Screen.setCursor(1, 1);
    // controls.controller.Screen.print("Auton done");
    // controls.controller.Screen.setCursor(2, 1);
    // controls.controller.Screen.print("%d:%.2f", angleDiff > 3,angleDiff);
    // controls.controller.Screen.setCursor(3, 1);
    // controls.controller.Screen.print("%.2f, %.2f", p.heading, inertial.heading());

    runningAuton = false;
  }
}
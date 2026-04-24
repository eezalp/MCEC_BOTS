#include "MCEC_Objects.h"

using namespace MCEC;
// CoSwervePod
  void CoSwervePod::SetPowers(float move, float rot, bool straight){
    if(abs2(rot) < 0.1f) rot = 0;

    if(abs2(move) < 0.01f){
      drive.stop(vex::brakeType::hold);
    }else{
      drive.spin(reversed ? vex::reverse : vex::forward, move, vex::pct);
    }
    if(abs2(rot) < 0.01f){
      axis.stop(vex::brakeType::hold);
    }else{
      axis.spin(vex::forward, rot, vex::pct);
    }
  }

  void CoSwervePod::GoToVector(Vector2 targ, bool canForward, float dt){
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

    if(abs2(move) < 0.01f){
      drive.stop(vex::brakeType::hold);
    }else{
      drive.spin(reversed ? vex::reverse : vex::forward, move, vex::pct);
    }
    if(abs2(rot) < 0.01f){
      axis.stop(vex::brakeType::hold);
    }else{
      axis.spin(vex::forward, rot, vex::pct);
    }
  }


  void CoSwervePod::SetPIDVariables(float P, float I, float D){
    rotationPID.SetVariables(P, I, D);
  }

  void CoSwervePod::Brake(vex::brakeType br){
    axis.stop(br);
    drive.stop(br);
  }

  float CoSwervePod::GetAngle(){
    float angle = (axis.position(vex::rotationUnits::deg) * axisRatio) + rotationOffset;// + (reversed ? 180 : 0);
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
    return angle;
  }

  bool CoSwervePod::IsMoving(){
    return (abs2(axis.velocity(vex::velocityUnits::pct)) <= 0.01f && abs2(drive.velocity(vex::velocityUnits::pct)) <= 0.01f);
  }

  void CoSwervePod::SetRotationOffset(float _off){
    rotationOffset = _off;
  }

  float CoSwervePod::CalcWub(float g){
    return 1;
  }
  Vector2 CoSwervePod::GetTraveled(float dt, float wub) {
    float heading = GetAngle() * (M_PI/180.0f);
    float cur = drive.position(vex::rotationUnits::deg);

    // return relativeVelocity;

    return Vector2(
        std::cos(heading),
        std::sin(heading)
    );
  }
//

// CoSwerveDrive
  void CoSwerveDrive::UpdatePositionUsingAccel(float dt){
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
  void CoSwerveDrive::UpdatePositionUsingEncoder(float dt){
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
  void CoSwerveDrive::UpdatePositionUsingOdom(float dt){
    // static float lastHeading = inertial.heading();
    // float heading = inertial.heading();
    // float dTheta = AngleDiff(heading, lastHeading) * (M_PI/180.0f);
    // Vector2 posChange = Vector2(
    //   forward.GetDeltaDistance() - (forward.offset.x * dTheta),
    //   lateral.GetDeltaDistance() - (lateral.offset.y * dTheta)
    // );

    // currentPos += posChange.Rotate(((heading + lastHeading)) / 2);
    // lastHeading = heading;
  }
  void CoSwerveDrive::UpdatePosition(float dt){
    // UpdatePositionUsingEncoder(dt);
    // UpdatePositionUsingAccel(dt);
    // UpdatePositionUsingOdom(dt);
  }
  void CoSwerveDrive::Drive(Vector2 driveVector, float rotationPower){
    static uint32_t lastTime = Brain.Timer.system();
    Vector2 FL_rot = Vector2( 1,  1) * rotationPower; // -1,  1
    Vector2 FR_rot = Vector2( 1, -1) * rotationPower; //  1,  1
    Vector2 BL_rot = Vector2(-1,  1) * rotationPower; // -1, -1
    Vector2 BR_rot = Vector2(-1, -1) * rotationPower; //  1, -1
    
    // Vector2 FL_rot(-FL_pos.y * rotationPower,  FL_pos.x * rotationPower);
    // Vector2 FR_rot(FR_pos.y * rotationPower,  -FR_pos.x * rotationPower);
    // Vector2 BL_rot(BL_pos.y * rotationPower,  -BL_pos.x * rotationPower);
    // Vector2 BR_rot(-BR_pos.y * rotationPower,  BR_pos.x * rotationPower);
    
    driveVector.Rotate(-inertial.heading());

    // Vector2 FL_target = driveVector + FL_rot;
    // Vector2 FR_target = driveVector + FR_rot;
    // Vector2 BL_target = driveVector + BL_rot;
    // Vector2 BR_target = driveVector + BR_rot;
    FL_rot += driveVector;
    FR_rot += driveVector;
    BL_rot += driveVector;
    BR_rot += driveVector;

    float maxMag = FL_rot.GetMagnitude();
    if(FR_rot.GetMagnitude() > maxMag) maxMag = FR_rot.GetMagnitude();
    if(BL_rot.GetMagnitude() > maxMag) maxMag = BL_rot.GetMagnitude();
    if(BR_rot.GetMagnitude() > maxMag) maxMag = BR_rot.GetMagnitude();
    
    if (maxMag > MAX_MODULE_OUTPUT) {
      float scale = MAX_MODULE_OUTPUT / maxMag;
      FL_rot *= scale;
      FR_rot *= scale;
      BL_rot *= scale;
      BR_rot *= scale;
    }
    
    uint32_t curTime = Brain.Timer.system();
    float dt = (curTime - lastTime) / 1000.0f;
    lastTime = curTime;

    frontLeft.GoToVector(FL_rot, canForward, dt);
    frontRight.GoToVector(FR_rot, canForward, dt);
    backLeft.GoToVector(BL_rot, canForward, dt);
    backRight.GoToVector(BR_rot, canForward, dt);
  }

  void CoSwerveDrive::Stop(vex::brakeType br){
    frontLeft.Brake(br); frontRight.Brake(br);
    backLeft.Brake(br);  backRight.Brake(br);
  }

  void CoSwerveDrive::AutonMove(Vector2 driveVector, float targetHeading, float dt){
    float power = autonMovePID.Update(driveVector.GetMagnitude(), dt);
    float angleDiff = AngleDiff(targetHeading, inertial.heading());
    float rotationPower = autonRotationPID.Update(angleDiff, dt);
    if(abs2(angleDiff) < 3) rotationPower = 0;
    driveVector = driveVector.Normalize();

    Drive(driveVector * power * 50, rotationPower * 30);
  }

  void CoSwerveDrive::SetRotationOffsets(float fl, float fr, float bl, float br){
    frontLeft.SetRotationOffset(fl); frontRight.SetRotationOffset(fr);
    backLeft.SetRotationOffset(bl); backRight.SetRotationOffset(br);
  }

  void CoSwerveDrive::FaceDirection(float direction){
    static RotationPID pid(0.04f, 0.0f, 0.004f);
    float curTime = (vex::timer::system());
    float lastTime = curTime;
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

  void CoSwerveDrive::SetAutonPIDVariables(float moveP, float moveI, float moveD, float rotationP, float rotationI, float rotationD){
    autonMovePID.SetVariables(moveP, moveI, moveD);
    autonRotationPID.SetVariables(rotationP, rotationI, rotationD);
  }

  void CoSwerveDrive::Initialize(){
    frontLeft.screen  =  frontLeft.rotationPID.screen = 1;
    frontRight.screen =  frontRight.rotationPID.screen = 4;
    backLeft.screen   =  backLeft.rotationPID.screen = 4;
    backRight.screen  =  backRight.rotationPID.screen = 4;

    frontLeft.rotationPID.Prime(frontLeft.GetAngle(), frontLeft.GetAngle());
    frontRight.rotationPID.Prime(frontRight.GetAngle(), frontRight.GetAngle());
    backLeft.rotationPID.Prime(backLeft.GetAngle(), backLeft.GetAngle());
    backRight.rotationPID.Prime(backRight.GetAngle(), backRight.GetAngle());

    frontLeft.relPos = Vector2(-trackwidth, wheelbase);
    frontRight.relPos = Vector2(trackwidth, wheelbase);
    backLeft.relPos = Vector2(-trackwidth, -wheelbase);
    backRight.relPos = Vector2(trackwidth, -wheelbase);

    isInitialized = true;
  }

  void CoSwerveDrive::GoToPoint(){
    static uint32_t lastTime = vex::timer::system();
    uint32_t curTime = vex::timer::system();
    float dt = (curTime - lastTime) / 1000.0f;
    // random non-zero number to prevent early exit of loop
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
        
        direction.x = p.point.x - currentPos.x;
        direction.y = p.point.y - currentPos.y;
        angleDiff = abs2(AngleDiff(p.heading, inertial.heading()));
        vex::this_thread::sleep_for(20);
      }

      Stop();
    }

    runningAuton = false;
  }
//
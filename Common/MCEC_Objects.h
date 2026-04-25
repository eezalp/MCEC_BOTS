
#pragma once
#ifndef MCEC_Objects_h
#define MCEC_Objects_h
#include "vex.h"
#include "Vectors.h"

#include <functional>
#include <vector>
#include <initializer_list>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstring>

#define IS_RED(color)  (0xFF0000 & (uint32_t)detectedColor) >> 16 == 0xff
#define IS_BLUE(color) (0x0000FF & (uint32_t)detectedColor) == 0xff

#define ABS(num) ((num < 0) ? (num * -1) : num)


typedef std::function<void()> ButtonCB;
//
//(14.5f + 9)
static const float wheelRad = 3.25f / 2;
static const float wheelDist = (10.0f + (5.0f/16.0f) + (12.5f) + (5.0f/16.0f)) / 2;
static const float wheelCirc = 2 * wheelRad * M_PI;

extern vex::competition comp;
extern vex::brain Brain;
extern vex::inertial inertial;

namespace MCEC{
  
  extern vex::mutex screenMutex;
  float abs2(float n);
  float AngleDiff(float a_1, float a_2);
  float WrapAngle(float a);

  
  template <typename T>
  T inline max2(T a, T b){
    return a > b ? a : b;
  }

  template <typename T>
  inline T Clamp(T a, T t, T b){
    return (a > t) ? t : ((a < b) ? b : a);
  }
  
  struct PathPoint{
    Vector2 point;
    float heading;
    float speed;
  };
  std::vector<PathPoint> pathFromFile(const char* filename);
  
  class RotationPID{
    public:
      RotationPID(float P, float I, float D) : m_P(P), m_I(I), m_D(D) { }
      RotationPID() : m_P(0), m_I(0), m_D(0) {}
      void SetVariables(float P, float I, float D);
      void SetTarget(float targ);
      void Prime(float curAngle, float targAngle);
      void Reset();
      bool AtTarget(float curAngle, float dt);
      float Update(float error, float dt);
      int screen = 4;
    private:    
      float m_P = 1, m_I = 1, m_D = 0.1f;
      float _target = 0;
      float _integral = 0;
      float _prevError = 0;
      float _maxOutput = 1.0f;
      float _integralMax = 0.3f;
  };
  class PID {
    public:
      PID(float P, float I, float D) : m_P(P), m_I(I), m_D(D) { }
      PID() : m_P(0), m_I(0), m_D(0) {}
      void SetVariables(float P, float I, float D);
      void SetTarget(float targ);
      void Prime(float curAngle, float targAngle);
      void Reset();
      float Update(float error, float dt);
    private:
      float m_P = 1, m_I = 1, m_D = 0.1f;
      float _target = 0;
      float _integral = 0;
      float _prevError = 0;
      float _maxOutput = 1.0f;
      float _integralMax = 0.3f;
  };
  class MotorGroup{
    public:
      std::vector<vex::motor> groupList;
      MotorGroup(){

      }
      template <class T>
      MotorGroup( std::initializer_list<T> list ) {
        for( auto elem : list ) {
          groupList.push_back(elem);
        }
      }
      void Spin(vex::directionType dir, float vel, vex::velocityUnits velUnits){
        for (auto motor : groupList){
          motor.spin(dir, vel, velUnits);
        }
      }
      float Position(vex::rotationUnits units){
        float pos = 0;
        for (auto motor : groupList){
          pos += motor.position(units);
        }
        return pos/groupList.size();
      }
      void SetVelocity(float vel, vex::percentUnits units){
        for (auto motor : groupList){
          motor.setVelocity(vel, units);
        }
      }
      void Stop(vex::brakeType bt){
        for (auto motor : groupList){
          motor.stop(bt);
        }
      }
      void SpinTo(float vel, vex::rotationUnits units, bool dir){
        for (auto motor : groupList){
          motor.spinTo(vel, units, dir);
        }
      }
      void ResetPosition(){
        for (auto motor : groupList){
          motor.resetPosition();
        }
      }

  };
  class Drivetrain{
    public:
    Drivetrain(){}
    RotationPID pid = RotationPID(1, 1, 0.1f);
    MotorGroup leftMotors;
    MotorGroup rightMotors;
    float leftMulti = 1;
    float rightMulti = 1;
      vex::inertial* inertial;

      float curPowerR, curPowerL, _heading;
      float wheelRadius = wheelRad, wheelDistance = wheelDist;
      int Lpower, Rpower;

      // @param joyX: input from -100 to 100
      // @param joyY: input from -100 to 100
      void ApplyPower(int lPow, int rPow);
      void Drive(int joyX, int joyY);
      // @param targ Angle in degrees of desired field rotation
      void Rotate(float targ);
      void Rotate(float rotation, float power);
      void DriveDist(float d, float sec);
      void DriveDist(float dL, float dR, float sec);
      void UpdateHeading();
      void SetInertial(vex::inertial* _inertial);
      void Stop(vex::brakeType br = vex::brakeType::brake);
      void SpinL(float);
      void SpinR(float);
      float ReadLeft();
      float ReadRight();
      void PowerLeft(int);
      void PowerRight(int);
      void SpinToR(float);
      void SpinToL(float);
      void ResetPositions();
      void SetSpeed(float speed = 50, vex::percentUnits units = vex::percentUnits::pct);
      void Spin(float revs, float power = 60);
    private:
  };

  class Drivetrain8 : public Drivetrain{
    public:
      Drivetrain8(
        int32_t portR1, int32_t portR2, 
        int32_t portR3, int32_t portR4, 
        int32_t portL1, int32_t portL2,
        int32_t portL3, int32_t portL4
      ) : 
      _mR1(portR1, vex::ratio6_1), _mR2(portR2, vex::ratio6_1), _mR3(portR3, vex::ratio6_1), _mR4(portR4, vex::ratio6_1),
      _mL1(portL1, vex::ratio6_1), _mL2(portL2, vex::ratio6_1), _mL3(portL3, vex::ratio6_1), _mL4(portL4, vex::ratio6_1)
      {
        leftMotors = MotorGroup({_mL1, _mL2, _mL3, _mL4});
        rightMotors = MotorGroup({_mR1, _mR2, _mR3, _mR4});
      }
    private:
      vex::motor _mR1, _mR2, _mR3, _mR4;
      vex::motor _mL1, _mL2, _mL3, _mL4;
  };
  class Drivetrain6 : public Drivetrain{
    public:
      vex::motor_group left;
      vex::motor_group right;
      Drivetrain6(
        int32_t portR1, int32_t portR2, int32_t portR3, 
        int32_t portL1, int32_t portL2, int32_t portL3
      ) :
      _mR1(portR1, vex::ratio6_1), _mR2(portR2, vex::ratio6_1), _mR3(portR3, vex::ratio6_1),
      _mL1(portL1, vex::ratio6_1), _mL2(portL2, vex::ratio6_1), _mL3(portL3, vex::ratio6_1)
      {
        leftMotors = MotorGroup({_mL1, _mL2, _mL3});
        rightMotors = MotorGroup({_mR1, _mR2, _mR3});
      }
    private:
      vex::motor _mR1, _mR2, _mR3;
      vex::motor _mL1, _mL2, _mL3;
  };
  struct Joystick{
    int x, y;
    void Set(int _x, int _y){
      SetX(_x);
      SetY(_y);
    }
    void SetX(int _x){
      x = _x;
    }
    void SetY(int _y){
      y = _y;
    }

    bool isMoved();
  };

  class Button{
    public:
      void Set(bool);
      void SetOnPress(ButtonCB _op){
        _onPress = _op;
      }
      void SetOnRelease(ButtonCB _or){
        _onRelease = _or;
      }
    private:
      bool _value;
      ButtonCB _onPress, _onRelease;
  };

  class Controller{
    public:
    Button 
      A, B, X, Y, 
      L1, L2, R1, R2, 
      LS, RS, 
      Up, Down, Left, Right;
    float exponent = 2;
    Joystick lStick, rStick;
    vex::controller controller;
    Controller(float _exponent = 2.0f) : controller(), exponent(_exponent) {}
    void Set();
    float JoystickCurve(float input);
  };

  class OdomPod{
    public:
      OdomPod(Vector2 _offset, int port, float _r) : offset(_offset), encoder(port, false), wheelRadius(_r) {
        firstPos = lastPos = encoder.angle();
      }
      
      float GetDeltaDistance(){
        float curPos = encoder.angle();
        float delta = AngleDiff(curPos, lastPos);

        lastPos = curPos;

        return delta * wheelRadius * (M_PI/180.0f);
      }
      float GetAbsDistance(){
        float curPos = encoder.angle();
        return AngleDiff(curPos, firstPos) * (M_PI/180.0f) * wheelRadius;
      }
      Vector2 offset;
      vex::rotation encoder;
    private:
      float lastPos;
      float firstPos;
      float wheelRadius;
  };

  class CoSwerveDrive;
  struct CoSwervePodSettings{
    int32_t axisPin, drivePin;
    float rotationPower = 100.0f;
    float axisGearRatio = 1;
    int maxPower = 100;
  };
  class CoSwervePod{
    public:
      int screen = 1;
      RotationPID rotationPID;
      Vector2 relPos;
      const static int MAX_MOTOR_POWER = 100;
      const static constexpr float MAX_ROTATION_POWER = 100.0f;
      const static constexpr float MAX_ROTATION_ANGLE = 90;
      bool atTarget = false;
      void SetPowers(float move, float rot, bool straight = false);
      void GoToVector(Vector2 targ, bool canForward = false, float dt = 0);
      void Brake(vex::brakeType br);
      float GetAngle();
      float CalcWub(float);
      Vector2 GetTraveled(float dt, float wub);
      void SetRotationOffset(float _off);
      void SetPIDVariables(float P, float I, float D);
      bool IsMoving();
      CoSwervePod(CoSwervePodSettings settings, CoSwerveDrive* drivetrain) : 
        axis(settings.axisPin), drive(settings.drivePin), 
        axisRatio(settings.axisGearRatio), master(drivetrain) 
      {}
      CoSwervePod(int32_t _axis, int32_t _drive, float axisGearRatio, CoSwerveDrive* drivetrain) : 
        axis(_axis), drive(_drive), 
        axisRatio(axisGearRatio), master(drivetrain) 
      {}

    private:
      const static Vector2 MOTOR_TOP_VECTOR;
      const static Vector2 MOTOR_BOT_VECTOR;
      float lastTime, axisRatio;
      vex::motor axis, drive;
      bool onShortest = false, reversed = false;
      float rotationOffset = 0.0f;
      float delTop, delBot;
      CoSwerveDrive* master;

  };
  class CoSwerveDrive{
    public:
      CoSwerveDrive(
        CoSwervePodSettings FL, CoSwervePodSettings FR,
        CoSwervePodSettings BL, CoSwervePodSettings BR,
        float _wheelBase, 
        float _trackWidth
      ) :
        frontLeft(FL, this), frontRight(FR, this),
        backLeft(BL, this), backRight(BR, this),
        wheelbase(_wheelBase), trackwidth(_trackWidth)
      {}
      CoSwerveDrive(
        int32_t FLa, int32_t FLd, float FLratio,
        int32_t FRa, int32_t FRd, float FRratio,
        int32_t BLa, int32_t BLd, float BLratio,
        int32_t BRa, int32_t BRd, float BRratio,
        float _wheelBase, 
        float _trackWidth
      ) :
        frontLeft(FLa, FLd, FLratio, this), frontRight(FRa, FRd, FRratio, this),
        backLeft(BLa, BLd, BLratio, this), backRight(BRa, BRd, BRratio, this),
        wheelbase(_wheelBase), trackwidth(_trackWidth)
      {}
      void Stop(vex::brakeType br = vex::brakeType::hold);
      void Drive(Vector2 driveVector, float rotationSpeed);
      void SetRotationOffsets(float, float, float, float);
      void GoToPoint();
      void FaceDirection(float direction);
      void UpdatePosition(float dt);
      void AutonMove(Vector2 driveVector, float targetHeading, float dt);
      void UpdatePositionUsingAccel(float);
      void UpdatePositionUsingEncoder(float);
      void UpdatePositionUsingOdom(float);
      void SetAutonPIDVariables(float, float, float, float, float, float);
      void Initialize();

      Vector2 currentPos;
      
      CoSwervePod frontLeft, frontRight, backLeft, backRight;
      std::vector<PathPoint> points;
      bool canForward = true;
      bool runningAuton = false;
    private:
      bool isInitialized = false;
      const static int MAX_MODULE_OUTPUT = 100;
      float wheelbase  = (12.75f + 11.75f) / 2;
      float trackwidth = (14.125f + 13.125f) / 2;
      PID autonMovePID;
      RotationPID autonRotationPID;
      Vector2 currentVel, currentAccel;
  };

  class DiffSwerveDrive;
  struct DiffSwervePodSettings{
    int32_t topPin, botPin;
    int32_t rotationPin;
    float rotationPower = 100.0f;
    int maxPower = 100, topDir, botDir;
    DiffSwervePodSettings(int32_t _topPin, vex::directionType _topDir, int32_t _botPin, vex::directionType _botDir, int32_t _rotationPin, float _rotationPower = 100, int _maxPower = 100) : 
      topPin(_topPin), topDir(static_cast<int>(_topDir)),
      botPin(_botPin), botDir(static_cast<int>(_botDir)),
      rotationPin(_rotationPin), rotationPower(_rotationPower),
      maxPower(_maxPower)
    {}
  };

  class DiffSwervePod{
    public:
      int screen = 1;
      int topDir = 1;
      int botDir = 1;
      RotationPID rotationPID;
      vex::rotation rotation;
      Vector2 relPos;
      const static int MAX_MOTOR_POWER = 100;
      const static constexpr float MAX_ROTATION_POWER = 100.0f;
      const static constexpr float MAX_ROTATION_ANGLE = 90;
      bool cosplineMode = false;
      bool atTarget = false;
      float motorRatio = 1;
      void SetPowers(float move, float rot, bool straight = false);
      void GoToVector(Vector2 targ, bool canForward = false, float dt = 0);
      void Brake(vex::brakeType br);
      float GetAngle();
      float CalcWub(float);
      Vector2 GetTraveled(float dt, float wub);
      void SetRotationOffset(float _off);
      void SetPIDVariables(float P, float I, float D);
      bool IsMoving();

      DiffSwervePod(int32_t t, int32_t b, int32_t r, DiffSwerveDrive* drivetrain) : top(t), bottom(b), rotation(r), master(drivetrain) {}

    private:
      const static Vector2 MOTOR_TOP_VECTOR;
      const static Vector2 MOTOR_BOT_VECTOR;
      float lastTime;
      vex::motor top, bottom;
      bool onShortest = false, reversed = false;
      float rotationOffset = 0.0f;
      float delTop, delBot;
      DiffSwerveDrive* master;
  };

  class DiffSwerveDrive {
    public:

      DiffSwerveDrive(
        DiffSwervePodSettings FL,
        DiffSwervePodSettings FR,
        DiffSwervePodSettings BL,
        DiffSwervePodSettings BR,
        Vector2 forwardOffset, int32_t forwardPort,
        Vector2 lateralOffset, int32_t lateralPort,
        float odomWheelDiameter,
        float _wheelBase = (12.75f + 11.75f) / 2, 
        float _trackWidth = (14.125f + 13.125f) / 2
      ) :
        frontLeft(FL.topPin, FL.botPin, FL.rotationPin, this), frontRight(FR.topPin, FR.botPin, FR.rotationPin, this),
        backLeft(BL.topPin, BL.botPin, BL.rotationPin, this), backRight(BR.topPin, BR.botPin, BR.rotationPin, this),
        forward(forwardOffset, forwardPort, odomWheelDiameter / 2),
        lateral(lateralOffset, lateralPort, odomWheelDiameter / 2),
        wheelbase(_wheelBase), trackwidth(_trackWidth)
      {
        frontLeft.topDir  = FL.topDir;
        frontLeft.botDir  = FL.botDir;
        
        frontRight.topDir = FR.topDir;
        frontRight.botDir = FR.botDir;
        
        backLeft.topDir   = BL.topDir;
        backLeft.botDir   = BL.botDir;
        
        backRight.topDir  = BR.topDir;
        backRight.botDir  = BR.botDir;
      }
      DiffSwerveDrive(
        int32_t FLt, int32_t FLb, int32_t FLr,
        int32_t FRt, int32_t FRb, int32_t FRr,
        int32_t BLt, int32_t BLb, int32_t BLr,
        int32_t BRt, int32_t BRb, int32_t BRr,
        Vector2 forwardOffset, int32_t forwardPort,
        Vector2 lateralOffset, int32_t lateralPort,
        float odomWheelDiameter,
        float _wheelBase = (12.75f + 11.75f) / 2, 
        float _trackWidth = (14.125f + 13.125f) / 2
      ) :
        frontLeft(FLt, FLb, FLr, this), frontRight(FRt, FRb, FRr, this),
        backLeft(BLt, BLb, BLr, this), backRight(BRt, BRb, BRr, this),
        forward(forwardOffset, forwardPort, odomWheelDiameter / 2),
        lateral(lateralOffset, lateralPort, odomWheelDiameter / 2),
        wheelbase(_wheelBase), trackwidth(_trackWidth)
      {}
      void Stop(vex::brakeType br = vex::brakeType::hold);
      void Drive(Vector2 driveVector, float rotationSpeed);
      void SetRotationOffsets(float, float, float, float);
      void GoToPoint();
      void FaceDirection(float direction);
      void UpdatePosition(float dt);
      void AutonMove(Vector2 driveVector, float targetHeading, float dt);
      void UpdatePositionUsingAccel(float);
      void UpdatePositionUsingEncoder(float);
      void UpdatePositionUsingOdom(float);
      void SetAutonPIDVariables(float, float, float, float, float, float);
      void Initialize();

      Vector2 currentPos;
      
      DiffSwervePod frontLeft, frontRight, backLeft, backRight;
      std::vector<PathPoint> points;
      bool canForward = true;
      bool runningAuton = false;
    private:
      bool isInitialized = false;
      const static int MAX_MODULE_OUTPUT = 100;
      float wheelbase  = (12.75f + 11.75f) / 2;
      float trackwidth = (14.125f + 13.125f) / 2;
      PID autonMovePID;
      RotationPID autonRotationPID;
      Vector2 currentVel, currentAccel;
      OdomPod forward, lateral;
  };

  float Lerp(float a, float b, float t);

}
extern MCEC::Controller controls;

#endif
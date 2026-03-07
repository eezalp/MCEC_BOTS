
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
  

  float abs2(float n);
  float AngleDiff(float a_1, float a_2);

  template <typename T>
  inline T Clamp(T a, T t, T b){
    return (a > t) ? t : ((a < b) ? b : a);
  }
  
  struct PathPoint{
    Vector2 point;
    float speed, heading;
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

    Joystick lStick, rStick;
    vex::controller controller;
    Controller() : controller() {}
    void Set();
  };

  class OdomPod{
    public:
      OdomPod(Vector2 _offset, int port, float _r) : offset(_offset), encoder(port), wheelRadius(_r) {
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
    private:
      vex::rotation encoder;
      float lastPos;
      float firstPos;
      float wheelRadius;
  };

  class SwervePod{
    public:
      int screen = 1;
      RotationPID rotationPID;
      vex::rotation rotation;
      Vector2 relPos;
      const static int MAX_MOTOR_POWER = 100;
      const static constexpr float MAX_ROTATION_POWER = 100.0f;
      const static constexpr float MAX_ROTATION_ANGLE = 90;
      bool cosplineMode = false;
      bool atTarget = false;
      float motorRatio = 1;
      void SetPowers(Vector2 power, bool straight = false);
      void GoToVector(Vector2 targ, bool canForward = false);
      void Brake(vex::brakeType br);
      float GetAngle();
      float CalcWub(float);
      Vector2 GetTraveled(float dt, float wub);
      void SetRotationOffset(float _off);
      void SetPIDVariables(float P, float I, float D);
      bool IsMoving();

      SwervePod(int32_t t, int32_t b, int32_t r) : top(t), bottom(b), rotation(r) {}

    private:
      const static Vector2 MOTOR_TOP_VECTOR;
      const static Vector2 MOTOR_BOT_VECTOR;
      float lastTime;
      vex::motor top, bottom;
      bool onShortest = false, reversed = false;
      float rotationOffset = 0.0f;
      float delTop, delBot;
  };

  class SwerveDrive {
    public:

      SwerveDrive(
        int32_t FLt, int32_t FLb, int32_t FLr,
        int32_t FRt, int32_t FRb, int32_t FRr,
        int32_t BLt, int32_t BLb, int32_t BLr,
        int32_t BRt, int32_t BRb, int32_t BRr,
        Vector2 forwardOffset, int32_t forwardPort,
        Vector2 lateralOffset, int32_t lateralPort,
        float odomWheelRadius
      ) :
        frontLeft(FLt, FLb, FLr), frontRight(FRt, FRb, FRr),
        backLeft(BLt, BLb, BLr), backRight(BRt, BRb, BRr),
        forward(forwardOffset, forwardPort, odomWheelRadius),
        lateral(lateralOffset, lateralPort, odomWheelRadius)
      {}
      void Stop(vex::brakeType br = vex::brakeType::coast);
      void Drive(Vector2 driveVector, float rotationSpeed);
      void SetRotationOffsets(float, float, float, float);
      void GoToPoint();
      void FaceDirection(float direction);
      void UpdatePosition(float dt);
      void AutonMove(PathPoint pp);
      void UpdatePositionUsingAccel(float);
      void UpdatePositionUsingEncoder(float);
      void UpdatePositionUsingOdom(float);

      Vector2 currentPos;
      
      SwervePod frontLeft, frontRight, backLeft, backRight;
      std::vector<PathPoint> points;
    private:
      const static int MAX_MODULE_OUTPUT = 100;
      bool cospline = false;
      float wheelbase  = (12.75f + 11.75f) / 2;
      float trackwidth = (14.125f + 13.125f) / 2;
      Vector2 currentVel, currentAccel;
      OdomPod forward, lateral;
  };

  float Lerp(float a, float b, float t);

}
extern MCEC::Controller controls;

#endif
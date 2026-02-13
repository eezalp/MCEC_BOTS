
#pragma once
#ifndef MCEC_Objects_h
#define MCEC_Objects_h
#include "vex.h"
#include "Vectors.h"

#include <functional>
#include <vector>
#include <initializer_list>
#include <cmath>

#define IS_RED(color)  (0xFF0000 & (uint32_t)detectedColor) >> 16 == 0xff
#define IS_BLUE(color) (0x0000FF & (uint32_t)detectedColor) == 0xff

#define ABS(num) ((num < 0) ? (num * -1) : num)

typedef std::function<void()> ButtonCB;
//
//(14.5f + 9)
static const float wheelRad = 3.25f / 2;
static const float wheelDist = (10.0f + (5.0f/16.0f) + (12.5f) + (5.0f/16.0f)) / 2;
static const float wheelCirc = 2 * wheelRad * M_PI;

float abs2(float n);
float AngleDiff(float a_1, float a_2);
extern vex::competition comp;
extern vex::brain Brain;

namespace MCEC{

    class RotationPID{
        public:
            RotationPID(float P, float I, float D) : m_P(P), m_I(I), m_D(D) { }
            void SetVariables(float P, float I, float D);
            void SetTarget(float targ);
            void Prime(float curAngle, float targAngle);
            bool AtTarget(float curAngle, float dt);
            float Update(float curAngle, float dt);
        private:        
            float m_P = 1, m_I = 1, m_D = 0.1f;
            float _target = 0;
            float _integral = 0;
            float _prevError = 0;
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

        bool isMoved(){
          return (x != 0 || y != 0);
        }
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

    class SwervePod{
        public:
            const static int MAX_MOTOR_POWER = 100;
            const static int MAX_ROTATION_POWER = 60;
            const static int ROTATION_ERROR = 5;
            void SetPowers(Vector2 power);
            void GoToVector(Vector2 targ);
            float GetPivot(Vector2 targ, float angle);
            void Brake(vex::brakeType br);

            SwervePod(int32_t t, int32_t b, int32_t r) : top(t), bottom(b), rotation(r) {}

        private:
            const static Vector2 MOTOR_TOP_VECTOR;
            const static Vector2 MOTOR_BOT_VECTOR;
            vex::motor top, bottom;
            vex::rotation rotation;
            bool onShortest = false, reversed = false;
    };

    class SwerveDrive {
        public:
            SwerveDrive(
                int32_t FLt, int32_t FLb, int32_t FLr,
                int32_t FRt, int32_t FRb, int32_t FRr,
                int32_t BLt, int32_t BLb, int32_t BLr,
                int32_t BRt, int32_t BRb, int32_t BRr
            ) :
              frontLeft(FLt, FLb, FLr), frontRight(FRt, FRb, FRr),
              backLeft(BLt, BLb, BLr), backRight(BRt, BRb, BRr)
            {}
            void Stop(vex::brakeType br = vex::brakeType::coast);
            void Drive(Vector2 driveVector, float rotationSpeed);
        private:
            SwervePod frontLeft, frontRight, backLeft, backRight;
            const static int MAX_SPEED = 100;
            
            float wheelbase  = 20.25f - 2 * (2.708f);
            float trackwidth = 20.25f - 2 * (2.708f);
    };

    float Lerp(float a, float b, float t);
}
#endif
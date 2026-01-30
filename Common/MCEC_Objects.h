
#pragma once
#ifndef MCEC_Objects_h
#define MCEC_Objects_h
#include "vex.h"

#include <functional>
#include <vector>
#include <initializer_list>

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

namespace MCEC{
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
    
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        float leftMulti = 1;
        float rightMulti = 1;

          float curPowerR, curPowerL, _heading;
          float wheelRadius = wheelRad, wheelDistance = wheelDist;
          int Lpower, Rpower;

          // @param joyX: input from -100 to 100
          // @param joyY: input from -100 to 100
          void ApplyPower(int lPow, int rPow);
          void Drive(int joyX, int joyY);
          void Rotate(float rotation, float power = 60);
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
          vex::inertial* inertial;
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

    float Lerp(float a, float b, float t);
}
#endif
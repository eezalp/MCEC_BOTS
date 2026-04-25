#pragma once
#include "vex.h"
#include <vector>
#include <cstdio>
#include <cstring>

using namespace vex;

extern vex::brain Brain;

struct Vector2 {
    float x;
    float y;

    Vector2(float _x, float _y);
    float mag();
    float operator*(Vector2& other) const;
    Vector2 operator*(int& scalar) const;
    Vector2 project(Vector2 other);

    Vector2& Vector2::rotate(float deg);
};

struct PID {
    float kP, kI, kD;
    float integral;
    float prevError;

    PID(float p, float i, float d);
    float update(float target, float actual);
    void reset();
};

struct PathPoint {
    float x, y, speed, heading;
};

enum DriveDirection { FORWARD, BACKWARD, LEFT, RIGHT };
double Lerp(double, double, double);
std::vector<PathPoint> pathFromFile(const char* filename);

class XDrive {


    motor Fright;
    Vector2 FrightV;

    motor Bright;
    Vector2 BrightV;

    motor Fleft;
    Vector2 FleftV;

    motor Bleft;
    Vector2 BleftV;

    PID headingPID;

    float targetHeading;

    float LF, LB, RF, RB;
    
    void setVel(double _LF, double _LB, double _RF, double _RB);

    void Stop();
    void driveForward(int);
    void turnInPlace(int);

    
    public:
    bool fieldOriented;
    inertial *imu;
    XDrive(int imuPort, int frP, int brP, int flP, int blP, gearSetting ratio);
    XDrive(inertial* _imu, int frP, int brP, int flP, int blP, gearSetting ratio);
    void setTarget(float controllerX, float controllerY, float turning);
    void runPath(const char* path);
    void UpdateMotorSpeeds();
    void move(float distance, DriveDirection dir = FORWARD, float AUTONSPEED=0.25f);
    void turnInPlace(float targetDeg);
    void movePID(float distance, DriveDirection dir, float AUTONSPEED);
};

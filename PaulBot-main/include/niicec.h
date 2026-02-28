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
    Vector2 project(Vector2 other);
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

    void setVel(double LF, double LB, double RF, double RB);

    void Stop();

    inertial imu;

public:
    XDrive(int imuPort, int frP, int brP, int flP, int blP, gearSetting ratio);
    void setTarget(float controllerX, float controllerY, float turning);
    void runPath(const char* path);
};

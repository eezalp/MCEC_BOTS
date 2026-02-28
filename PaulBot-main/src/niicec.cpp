#include "../include/niicec.h"

// Vector2

Vector2::Vector2(float _x, float _y) : x(_x), y(_y) {}

float Vector2::mag() {
    return hypot(x, y);
}

float Vector2::operator*(Vector2& other) const {
    return other.x * x + other.y * y;
}

Vector2 Vector2::project(Vector2 other) {
    float scaleFactor = (*this * other) / pow(mag(), 2);
    return Vector2(scaleFactor * x, scaleFactor * y);
}

// PID

PID::PID(float p, float i, float d) : kP(p), kI(i), kD(d), integral(0), prevError(0) {}

float PID::update(float target, float actual) {
    float error = target - actual;
    integral += error;
    float output = kP * error + kI * integral + kD * (error - prevError);
    prevError = error;
    return output;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}

// pathFromFile

std::vector<PathPoint> pathFromFile(const char* filename) {
    std::vector<PathPoint> points;

    int32_t size = Brain.SDcard.size(filename);
    if (size <= 0) return points;

    uint8_t* buf = new uint8_t[size + 1];
    Brain.SDcard.loadfile(filename, buf, size);
    buf[size] = '\0';

    char* line = strtok((char*)buf, "\n");
    while (line != nullptr) {
        PathPoint p = {0, 0, 0, 0};
        sscanf(line, "%f,%f,%f,%f", &p.x, &p.y, &p.speed, &p.heading);
        points.push_back(p);
        line = strtok(nullptr, "\n");
    }

    delete[] buf;
    return points;
}

// XDrive

XDrive::XDrive(int imuPort, int frP, int brP, int flP, int blP, gearSetting ratio)
    : Fright(frP, ratio, true), Bright(brP, ratio, true), Fleft(flP, ratio, false), Bleft(blP, ratio, false),
      FrightV(-sqrt(2)/2, sqrt(2)/2), BrightV(sqrt(2)/2, sqrt(2)/2),
      FleftV(sqrt(2)/2, sqrt(2)/2), BleftV(-sqrt(2)/2, sqrt(2)/2),
      headingPID(0.8, 0.0, 0.05), targetHeading(0), imu(imuPort) {}

void XDrive::setVel(double LF, double LB, double RF, double RB) {
    Fleft.spin(LF >= 0 ? fwd : reverse, fabs(LF), pct);
    Bleft.spin(LB >= 0 ? fwd : reverse, fabs(LB), pct);
    Fright.spin(RF >= 0 ? fwd : reverse, fabs(RF), pct);
    Bright.spin(RB >= 0 ? fwd : reverse, fabs(RB), pct);
}

void XDrive::Stop(){
  headingPID.reset();
  targetHeading = imu.rotation();
  Fleft.stop(vex::brakeType::hold);
  Bleft.stop(vex::brakeType::hold);
  Fright.stop(vex::brakeType::hold);
  Bright.stop(vex::brakeType::hold);
}

void XDrive::setTarget(float controllerX, float controllerY, float turning) {
    Vector2 target(controllerX, controllerY);
    float correction = 0;
    if (turning != 0) {
        targetHeading = imu.rotation();
    } else {
        correction = headingPID.update(targetHeading, imu.rotation());
    }

    if (controllerX != 0 || controllerY != 0 || turning != 0) {
        setVel(
            FleftV * target + turning + correction,
            BleftV * target + turning + correction,
            FrightV * target - turning - correction,
            BrightV * target - turning - correction
        );
    } else {
        Stop();
    }
}

void XDrive::runPath(const char* path) {
    auto points = pathFromFile(path);
    for (int i = 0; i < (int)points.size() - 1; i++) {
        float dx = points[i+1].x - points[i].x;
        float dy = points[i+1].y - points[i].y;
        float dist = hypot(dx, dy);
        if (dist == 0) continue;

        if (points[i].heading != 0)
            targetHeading = points[i].heading;

        //200 is a magic number from the ratio
        float vx = (dx / dist) * (points[i].speed / 200.0f);
        float vy = (dy / dist) * (points[i].speed / 200.0f);

        setTarget(vx, vy, 0);
        wait(15, msec);
    }
    setVel(0, 0, 0, 0);
}

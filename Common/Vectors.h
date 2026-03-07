#ifndef Vectors_h
#define Vectors_h


#include <cmath>

struct Vector2{
  public:
    float x, y;

    Vector2(float _x, float _y) : x(_x), y(_y) { }
    Vector2(Vector2 direction, float magnitude);
    Vector2(Vector2 final, Vector2 initial);
    Vector2() : x(0), y(0) { }
    float GetMagnitude();

  // @param deg: angle in degrees
    Vector2& Rotate(float deg);

    float GetAngle();

    Vector2& Normalize();

    // @param destination: The destination vector
    static Vector2 Project(Vector2 source, Vector2 destination){
      Vector2 res = destination.Normalize() * ( (source * destination) / destination.GetMagnitude());
      return res; 
    }
    Vector2& Project(Vector2 destination);
    // Operators
    Vector2& operator*=(const float scalar);
    Vector2& operator/=(const float scalar);
    Vector2& operator+=(const Vector2 b);
    
    // Constant Operators
    Vector2 operator*(const float scalar) const;
    Vector2 operator/(const float scalar) const;
    Vector2 operator+(const Vector2 b) const;
    Vector2 operator-(const Vector2 b) const;
    // Dot product
    float operator*(const Vector2 b) const;
    // Cross product
    float operator%(const Vector2 b) const;
    //
    static const Vector2 zero, up, down, left, right;
    private:
};


struct Vector3{
  public:
    float x, y, z;
  private:
};

#endif
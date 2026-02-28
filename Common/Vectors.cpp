#include <Vectors.h>

//Vector 2
  
  const Vector2 Vector2::zero  = { 0,  0};
  const Vector2 Vector2::up    = { 0,  1}; 
  const Vector2 Vector2::down  = { 0, -1}; 
  const Vector2 Vector2::left  = {-1,  0};
  const Vector2 Vector2::right = { 1,  0};
  Vector2::Vector2(Vector2 direction, float magnitude){
    direction.Normalize();
    direction *= magnitude;
    x = direction.x;
    y = direction.y;
  }
  Vector2::Vector2(Vector2 final, Vector2 initial){
    x = final.x - initial.x;
    y = final.y - initial.y;
  }

  float Vector2::GetMagnitude(){
    return std::sqrt(x*x + y*y);
  }

  Vector2& Vector2::Rotate(float deg){
    float r = deg * (M_PI / 180.0f);
    float newX = x * std::cos(r) - y * std::sin(r);
    y = x * std::sin(r) + y * std::cos(r);
    x = newX;
    return *this;
  }

  float Vector2::GetAngle(){
    return std::atan2(y, x) * (180.0f/M_PI);
  }

  Vector2& Vector2::Normalize(){
    float mag = GetMagnitude();
    x /= mag;
    y /= mag;
    return *this;
  }

  

  Vector2& Vector2::Project(Vector2 destination){
    Vector2 source = Vector2(x, y);
    source = destination.Normalize() * ( (source * destination) / destination.GetMagnitude());
    x = source.x;
    y = source.y;
    return *this;
  }

  // Operators
  Vector2& Vector2::operator*=(const float scalar){
    this->x *= scalar;
    this->y *= scalar;
    return *this;
  };

  Vector2& Vector2::operator/=(const float scalar){ 
    this->x /= scalar;
    this->y /= scalar;
    return *this;
  }
  Vector2& Vector2::operator+=(const Vector2 b){
    this->x += b.x;
    this->y += b.y;
    return *this;
  };
  
  // Constant Operators
  Vector2 Vector2::operator*(const float scalar) const{
    Vector2 res(this->x, this->y);
    res.x *= scalar;
    res.y *= scalar;
    return res;
  };
  Vector2 Vector2::operator/(const float scalar) const{ 
    Vector2 res(this->x, this->y);
    res.x /= scalar;
    res.y /= scalar;
    return res;
  }
  Vector2 Vector2::operator+(const Vector2 b) const{
    Vector2 res(this->x, this->y);
    res.x += b.x;
    res.y += b.y;
    return res;
  }
  Vector2 Vector2::operator-(const Vector2 b) const{
    Vector2 res(this->x, this->y);
    res.x -= b.x;
    res.y -= b.y;
    return res;
  }
  // Dot product
  float Vector2::operator*(const Vector2 b) const{
    return (x * b.x) + (y * b.y);
  }
  // Cross product
  float Vector2::operator%(const Vector2 b) const{
    return (x * b.y) - (b.x * y);
  }
//
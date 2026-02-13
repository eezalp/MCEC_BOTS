#ifndef Vectors_h
#define Vectors_h


#include <cmath>

struct Vector2{
    public:
        float x, y;

        Vector2(float _x, float _y) : x(_x), y(_y) { }
        Vector2(Vector2 direction, float magnitude){
            direction.Normalize();
            direction *= magnitude;
            x = direction.x;
            y = direction.y;
        }
        Vector2() : x(0), y(0) { }

        float inline GetMagnitude(){
            return std::sqrt(x*x + y*y);
        }

        float inline GetAngle(){
            return std::atan2(x, y) * (180.0f/M_PI);
        }

        Vector2& Normalize(){
            float mag = GetMagnitude();
            x /= mag;
            y /= mag;
            return *this;
        }

        // @param destination: The destination vector
        static Vector2 Project(Vector2 source, Vector2 destination){
            Vector2 res = destination.Normalize() * ( (source * destination) / destination.GetMagnitude());
            return res; 
        }
        Vector2& Project(Vector2 destination){
            Vector2 source = Vector2(x, y);
            source = destination.Normalize() * ( (source * destination) / destination.GetMagnitude());
            x = source.x;
            y = source.y;
            return *this;
        }

        // Operators
            Vector2& operator*=(const float scalar){
                this->x *= scalar;
                this->y *= scalar;
                return *this;
            };
            Vector2& operator/=(const float scalar){ 
                this->x /= scalar;
                this->y /= scalar;
                return *this;
            }
        //
        // Constant Operators
            Vector2 operator*(const float scalar) const{
                Vector2 res(this->x, this->y);
                res.x *= scalar;
                res.y *= scalar;
                return res;
            };
            Vector2 operator/(const float scalar) const{ 
                Vector2 res(this->x, this->y);
                res.x /= scalar;
                res.y /= scalar;
                return res;
            }
            Vector2 operator+(const Vector2 b) const{
                Vector2 res(this->x, this->y);
                res.x += b.x;
                res.y += b.y;
                return res;
            }
            Vector2 operator-(const Vector2 b) const{
                Vector2 res(this->x, this->y);
                res.x -= b.x;
                res.y -= b.y;
                return res;
            }

            // Dot product
            float operator*(const Vector2 b) const{
                return (x * b.x) + (y * b.y);
            }
            // Cross product
            float operator%(const Vector2 b) const{
                return (x * b.y) - (b.x * y);
            }
        //

    private:
};

struct Vector3{
    public:
        float x, y, z;
    private:
};

#endif
//
// Created by Kevin Yu on 9/20/15.
//

#include "falton/math/Vector3.h"

 namespace falton {

     Vector3::Vector3(): x(0), y(0), z(0) {}

     Vector3::Vector3(real x,real y,real z): x(x), y(y), z(z) {}

     real Vector3::dot(const Vector3& rhs) const {
        return x * rhs.x + y *rhs.y + z * rhs.z;
     }

     Vector3 Vector3::cross(const Vector3& rhs) const{
        return Vector3(y * rhs.z - z * rhs.y,
                    z * rhs.x - x * rhs.z,
                    x * rhs.y - y * rhs.x);
     }

     Vector3 Vector3::operator+(const Vector3 &rhs) const {
        return Vector3(x+rhs.x,y+rhs.y,z+rhs.z);
     }

     Vector3 Vector3::operator-(const Vector3 &rhs) const {
        return Vector3(x-rhs.x,y-rhs.y,z-rhs.z);
     }

     Vector3 Vector3::operator*(real scale) const {
         return Vector3(x * scale, y * scale, z *scale);
     }

     void Vector3::cross_update(const Vector3& rhs) {
         int new_x,new_y,new_z;
         new_x = y * rhs.z - z * rhs.y;
         new_y = z * rhs.y - y * rhs.z;
         new_z = x * rhs.y - y * rhs.x;

         x = new_x;
         y = new_y;
         z = new_z;
     }

    Vector3& Vector3::operator+=(const Vector3 &rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;
        return *this;
    }

    void Vector3::operator-=(const Vector3 &rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;
    }

    bool Vector3::operator==(const Vector3 &rhs) const {
        return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
    }

    void Vector3::operator*=(real scale) {
        x *= scale;
        y *= scale;
        z *= scale;
    }

     void Vector3::zero() {
         x = 0;
         y = 0;
         z = 0;
     }

    void Vector3::normalize() {
        real length = magnitude();
        if (length > 0) (*this) *= ((real)1/length);
    }

    real Vector3::magnitude() const {
        return real_sqrt(square_magnitude());
    }

    real Vector3::square_magnitude() const {
        return x * x + y * y + z * z;
    }

 }
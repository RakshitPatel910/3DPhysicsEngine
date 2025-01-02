#pragma once

#include<cmath>

class Vector3
{
private:
    float x, y, z;
public:
    Vector3(): x(0), y(0), z(0) {}

    Vector3(float x, float y, float z): x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& b) const {
        return Vector3( x + b.x, y + b.y, z + b.z );
    }

    Vector3 operator-(const Vector3& b) const {
        return Vector3( x - b.x, y - b.y, z - b.z );
    }

    Vector3 operator*(const float scalar) const {
        return Vector3( scalar * x, scalar * y, scalar * z );
    }

    Vector3 operator/(const float scalar) const {
        return Vector3( x / scalar, y / scalar, z / scalar );
    }

    Vector3& operator+=(const Vector3& b) {
        x += b.x;
        y += b.y;
        z += b.z;
        return *this;
    }

    bool operator<(const Vector3& other) const {
        return (x < other.x) && (y < other.y) && (z < other.z);
    }

    bool operator>(const Vector3& other) const {
        return (x > other.x) && (y > other.y) && (z > other.z);
    }

    bool operator==(const Vector3& other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    void printV() const {
        std::cout << x << ", " << y << ", " << z << '\n';
    }

    float dot(const Vector3& b) const {
        return x * b.x + y * b.y + z * b.z;
    }

    Vector3 cross(const Vector3& b) const {
        return Vector3(
            y * b.z - z * b.y,
            z * b.x - x * b.z,
            x * b.y - y * b.x
        );
    }

    float length() const { // Norm2
        return std::sqrt( x*x + y*y + z*z );
    }

    float lengthSq() const {
        return x*x + y*y + z*z ;
    }

    Vector3 normalized() const {
        float len = length();

        return len > 0 ? *this * (1.0f / len) : Vector3();
    }

    void negate() const {
        x = -x;
        y = -y;
        z = -z;
    }

    float angleBetween(const Vector3& b) const {
        float mag_a = length();
        float mag_b = b.length();

        if( mag_a == 0 || mag_b == 0 ) return 0.0f;

        float dotProd = dot(b);
        float cosTheta = dotProd / (mag_a * mag_b);

        //clamping for making acos use safe
        cosTheta = std::fmax(-1.0f, std::fmin(1.0f, cosTheta));

        return std::acos(cosTheta);
    }
};


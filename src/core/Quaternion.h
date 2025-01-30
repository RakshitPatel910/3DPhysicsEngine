#pragma once

#include<cmath>
#include "Vector3.h"
#include "Matrix4.h"

class Quaternion
{
private:
    float w, x, y, z;
public:
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    // Quaternion q(3.14159265359f / 2.0f, 0.0f, 1.0f, 0.0f) : quaternion representing a rotation of 90 degrees around the Y-axis
    // Quaternion(float angle, float ax, float ay, float az){
    //     float halfTheta = angle * 0.5f;
    //     float sinHalfTheta = std::sin(halfTheta);

    //     w = std::cos(halfTheta);
    //     x = ax * sinHalfTheta;
    //     y = ay * sinHalfTheta;
    //     z = az * sinHalfTheta;
    // }

    Quaternion(float angle, float ax, float ay, float az)
    : w(std::cos(angle * 0.5f)),
      x(ax * std::sin(angle * 0.5f)),
      y(ay * std::sin(angle * 0.5f)),
      z(az * std::sin(angle * 0.5f)) 
    {}
    
    Quaternion(float angle, const Vector3& ang_v)
    : w(angle),           
      x(ang_v.getX()),      
      y(ang_v.getY()),      
      z(ang_v.getZ()) 
    {}

    Quaternion operator*(const Quaternion& q) const {
        Quaternion q_final;

        q_final.w = w*q.w - x*q.x - y*q.y - z*q.z;
        q_final.x = w*q.x + x*q.w + y*q.z - z*q.y;
        q_final.y = w*q.y + y*q.w - x*q.z + q.x*z;
        q_final.z = w*q.z + z*q.w + x*q.y - q.x*y;

        return q_final;
    }

    Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    float getW() const { return w; }

    void printQ() const {
        std::cout << w << ", " << x << ", " << y << ", " << z << '\n';
    }

    void normalize(){
        float q_len = std::sqrt( w*w + x*x + y*y + z*z );

        w = w / q_len; 
        x = x / q_len; 
        y = y / q_len; 
        z = z / q_len; 
    }

    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Matrix4 toMatrix4() const {
        Matrix4 mat;

        mat.m[0] = 1.0f - 2.0f * (y*y + z*z);
        mat.m[1] = 2.0f * (x*y - z*w);
        mat.m[2] = 2.0f * (x*z + y*w);
        mat.m[3] = 0.0f;

        mat.m[4] = 2.0f * (x*y + z*w);
        mat.m[5] = 1.0f - 2.0f * (x*x + z*z);
        mat.m[6] = 2.0f * (y*z - x*w);
        mat.m[7] = 0.0f;

        mat.m[8] = 2.0f * (x*z - y*w);
        mat.m[9] = 2.0f * (y*z + x*w);
        mat.m[10] = 1.0f - 2.0f * (x*x + y*y);
        mat.m[11] = 0.0f;

        mat.m[12] = 0.0f;
        mat.m[13] = 0.0f;
        mat.m[14] = 0.0f;
        mat.m[15] = 1.0f;

        return mat;
    }

    Vector3 rotateVec(const Vector3& vec) const {
        Quaternion p(0, v.x, v.y, v.z);
        Quaternion q_conj = conjugate();
        Quaternion rotated = (*this * p) * q_conj;

        return Vector3(rotated.x, rotated.y, rotated.z);
    }

    static Quaternion fromEuler(float yaw, float pitch, float roll) {
        Quaternion qy(yaw, Vector3(0.0f, 1.0f, 0.0f));
        Quaternion qp(pitch, Vector3(1.0f, 0.0f, 0.0f));
        Quaternion qr(roll, Vector3(0.0f, 0.0f, 1.0f));

        return qy * qp * qr;
    }   
};


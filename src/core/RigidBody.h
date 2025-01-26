#pragma once

#include<cmath>
#include<string>
#include "Vector3.h"
#include "Matrix4.h"
#include "Quaternion.h"


class BoxShape {
public:
    float width;
    float height;
    float depth;
    float mass;
    
    Matrix4 inertiaTensor;

    BoxShape(float w = 1.0f, float h = 1.0f, float d = 1.0f, float m = 1.0f)
        : width(w), height(h), depth(d), mass(m) {
        calcInertia();
    }

    void calcInertia() {
        // The inertia tensor for a box (centered at origin and aligned with axes)
        float I_xx = (mass / 12.0f) * (height * height + depth * depth);
        float I_yy = (mass / 12.0f) * (width * width + depth * depth);
        float I_zz = (mass / 12.0f) * (width * width + height * height);

        // Set inertia tensor components in the diagonal matrix
        inertiaTensor = Matrix4();
        inertiaTensor.m[0] = I_xx;  // I_xx
        inertiaTensor.m[5] = I_yy;  // I_yy
        inertiaTensor.m[10] = I_zz; // I_zz
    }
};


class RigidBody
{
private:
    BoxShape shape;

    Vector3 pos;
    Vector3 v;
    std::string m_name;
    Vector3 ang_v;
    Vector3 force;
    Vector3 torque;

    Quaternion q;

    Matrix4 tr_mat;
    
    Matrix4 inertiaTensor;
    Matrix4 inverseInertiaTensor;


public:
    RigidBody(
        const BoxShape& shape = BoxShape(),
        const Vector3& pos = Vector3(),
        Vector3 v = Vector3(),
        std::string m_name = "",
        Vector3 ang_v = Vector3(),
        Vector3 force = Vector3(),
        Vector3 torque = Vector3(),
        Quaternion q = Quaternion()
        // Matrix4 tr_mat = Matrix4()
    ) : shape(shape), pos(pos), v(v), m_name(m_name), ang_v(ang_v), force(force), torque(torque), q(q) {
        updateInertiaTensor();
        updateTransform();
    }


    Vector3 getPosition() const { return pos; }
    Vector3 getVelocity() const { return v; }
    Vector3 getAngularVelocity() const { return ang_v; }
    Vector3 getForce() const { return force; }
    Vector3 getTorque() const { return torque; }
    Quaternion getOrientation() const { return q; }
    std::string getName() const { return m_name; }


    Matrix4 getTransformMatrix() const {
        return tr_mat;
    }

    Matrix4 getInertiaTensor() const {
        return inertiaTensor;
    }

    Matrix4 getInverseInertiaTensor() const {
        return inverseInertiaTensor;
    }

    Vector3 globalToLocalVector(const Vector3& v){
        q.normalize();
        Matrix4 matT = q.toMatrix4().transpose();

        matT.m[12] += pos.getX();
        matT.m[13] += pos.getY();
        matT.m[14] += pos.getZ();

        matT = matT.fromGlm( glm::inverse(matT.toGlm()) );

        return matT.transformVec(v);
    }

    Vector3 localToGlobalVector(const Vector3& v){
        q.normalize();
        Matrix4 mat = q.toMatrix4();

        mat.m[12] += pos.getX();
        mat.m[13] += pos.getY();
        mat.m[14] += pos.getZ();

        return mat.transformVec(v);
    }

    void updateInertiaTensor(){
        Matrix4 localInertia = shape.inertiaTensor; // Use shape's local inertia tensor
        Matrix4 rotationMat = q.toMatrix4();

        // Compute world inertia tensor: R * I_local * R^T
        inertiaTensor = rotationMat * localInertia * rotationMat.transpose();
        inverseInertiaTensor = inertiaTensor.inverse();
    }

    void updateTransform(){
        Matrix4 translationMat = Matrix4::getTranslationMatrix(pos);
        Matrix4 rotationMat = q.toMatrix4();
        Matrix4 scalingMat = Matrix4::getScalingMAtrix(Vector3(shape.width, shape.height, shape.depth));

        // tr_mat = translationMat * rotationMat * scalingMat;
        tr_mat = scalingMat * rotationMat * translationMat;
    }

    void applyForce(const Vector3& appliedForce, const Vector3& pt ){
        force += appliedForce; 
        
        // torque = r x f
        Vector3 r = pt - pos;
        // std::cout << pt.getX() << ", " << pt.getY() << ", " << pt.getZ() << std::endl;
        torque += r.cross(appliedForce);
        // pt.printV();
        // pos.printV();
        // r.printV();
        // torque.printV();
    }

    void integratePhysics(float dt){
        Vector3 a = force / shape.mass;
        v = v + a * dt;
        pos = pos + v * dt;

        // std::cout << force.getX() << ", " << force.getY() << ", " << force.getZ() << std::endl;

        updateInertiaTensor();
        Vector3 ang_a = inverseInertiaTensor.transformVec(torque);
        ang_v = ang_v + ang_a * dt;
        // ang_v.printV();

        // dq / dt = 0.5 * q * ang_v
        float dang_v = ang_v.length();
        Quaternion dq = Quaternion(dang_v, ang_v.getX(), ang_v.getY(), ang_v.getZ());
        // std::cout << ang_v.getX() << ", " << ang_v.getY() << ", " << ang_v.getZ() << std::endl;

        // dq = dq * q * 0.5f * dt;
        dq = q * dq * 0.5f * dt;
        q = q + dq;
        // q.printQ();
        q.normalize();

        force = Vector3();
        torque = Vector3();

        updateTransform(); //updateTransform() recalculates the transformation matrix every frame. If you don't use it frequently, consider deferring this calculation until explicitly required (e.g., during rendering or collision detection).
    }

};


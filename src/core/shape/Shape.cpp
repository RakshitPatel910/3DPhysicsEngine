#include "Shape.h"
#include "../MeshLibrary.h"

#include <vector>

Sphere::Sphere(const float radius) 
    : m_radius(radius) {
        mesh = &MeshLibrary::getSphereMesh();
        scaleFactor = Vector3(radius, radius, radius);
    }

Vector3 Sphere::Support(const Vector3& dir) const {
    return dir.normalized() * m_radius;
}

void Sphere::calculateInertiaTensor(float mass) {
    inertiaTensor = Matrix4();

    inertiaTensor.m[0] = inertiaTensor.m[5] = inertiaTensor.m[10] = (2.0f / 5.0f) * mass * m_radius * m_radius;
}

Matrix4 Sphere::getInertiaTensor() const {
    return inertiaTensor;
}

Cube::Cube(const Vector3& halfext) 
    : m_halfext(halfext) {
        mesh = &MeshLibrary::getCubeMesh();
        scaleFactor = halfext * 2;
    }

Vector3 Cube::Support(const Vector3& dir) const {
    Vector3 vec = Vector3(
        m_halfext.getX() * (dir.getX() >= 0 ? 1 : -1),
        m_halfext.getY() * (dir.getY() >= 0 ? 1 : -1),
        m_halfext.getZ() * (dir.getZ() >= 0 ? 1 : -1)
    );

    return vec;
}

void Cube::calculateInertiaTensor(float mass) {
    float w = m_halfext.getX() * 2.0f;
    float h = m_halfext.getY() * 2.0f;
    float d = m_halfext.getZ() * 2.0f;
    float c = 1.0f / 12.0f;


    // Calculate inertia tensor components for a cuboid
    float I_xx = c * mass * (h * h + d * d);
    float I_yy = c * mass * (w * w + d * d);
    float I_zz = c * mass * (w * w + h * h);

    inertiaTensor = Matrix4();
    inertiaTensor.m[0] = I_xx;  // I_xx
    inertiaTensor.m[5] = I_yy;  // I_yy
    inertiaTensor.m[10] = I_zz; // I_zz
}

Matrix4 Cube::getInertiaTensor() const {
    return inertiaTensor;
}

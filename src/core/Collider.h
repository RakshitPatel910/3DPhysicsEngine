#pragma once

#include "Vector3.h"
#include "Matrix4.h"
#include "RigidBody.h"

class Collider
{
public:
    float m_mass;
    Matrix4 m_localInertiaTensor;
    Vector3 m_localcentroid;
    RigidBody* m_body;

    RigidBody* getRigidBody() const {
        return m_body;
    }
};


#pragma once

#include "Vector3.h"
#include "Matrix4.h"

class Collider
{
public:
    float mass;
    Matrix4 localInertiaTensor;
    Vector3 localcentroid;
};


#pragma once

#include<Eigen/Dense>

#include "Vector3.h"
#include "Matrix4.h"
#include "Collider.h"
#include "RigidBody.h"

class Constraint
{
public:
    Collider* colliderA;
    Collider* colliderB;

    
};


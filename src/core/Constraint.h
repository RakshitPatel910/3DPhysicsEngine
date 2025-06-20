#pragma once

#include<iostream>
#include<memory>
#include<Eigen/Dense>

#include "Vector3.h"
#include "Matrix4.h"
#include "Collider.h"
#include "RigidBody.h"
#include "EPA.h"

class Constraint
{
public:
    Collider* colliderA;
    Collider* colliderB;

    Eigen::Matrix<float, 1, 12> J; // Jacobian ==> has Va Wa Vb Wb
    Eigen::Matrix<float, 12, 12> M_inv; // Inverse Mass Matrix
    Eigen::Matrix<float, 12, 1> Minv_Jtr; // M^-1 * J^T // Catto_B

    float effectiveMass;

    Constraint(Collider* colliderA, Collider* colliderB) : 
        colliderA(colliderA), 
        colliderB(colliderB) 
    {
        calcInverseMassMatrix();
    }

    void calcInverseMassMatrix();

    virtual void calcJacobian() = 0;
    virtual float solveConstraint(
    // virtual void solveConstraint(
        float time,
        int iter
        // std::shared_ptr<ContactData> contactData
    ) = 0;
};


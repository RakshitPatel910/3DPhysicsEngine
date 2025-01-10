#pragma once

#include<Eigen/Dense>

#include "EPA.h"
#include "Constraint.h"

class ContactConstraint :: public Constriant
{
public:
    ContactData* contactData;

    ContactConstraint(Collider & colliderA, Collider & colliderB) : Constraint(colliderA, colliderB) {}

    virtual void calcJacobian() override;
    virtual void solveConstraint(
        float time,
        std::shared_ptr<ContactConstraint> contact
    ) override;
}
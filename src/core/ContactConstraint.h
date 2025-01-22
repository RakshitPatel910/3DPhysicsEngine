#pragma once

#include<Eigen/Dense>

#include "EPA.h"
#include "Constraint.h"

class ContactConstraint : public Constraint
{
public:
    ContactData* contactData;

    ContactConstraint(Collider* colliderA, Collider* colliderB) : Constraint(colliderA, colliderB) {}

    virtual void calcJacobian() override;
    virtual void solveConstraint(
        float time,
        std::shared_ptr<ContactData> contact
    ) override;
};
#pragma once

#include<Eigen/Dense>

#include "EPA.h"
#include "Constraint.h"

#include <memory>

class ContactConstraint : public Constraint
{
public:
    ContactData* contactData;
    // std::unique_ptr<ContactData> contactData;

    ContactConstraint(Collider* colliderA, Collider* colliderB, ContactData* cd) : Constraint(colliderA, colliderB), contactData(cd) {}
    // ContactConstraint(Collider* colliderA, Collider* colliderB) : Constraint(colliderA, colliderB) {}

    virtual void calcJacobian() override;
    virtual float solveConstraint(
        float time,
        int iter
        // std::shared_ptr<ContactData> contact
    ) override;
};
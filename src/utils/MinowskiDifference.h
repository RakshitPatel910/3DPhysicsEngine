#pragma once

#include "../core/Vector3.h"
#include "../core/RigidBody.h"
#include "../core/Collider.h"

class MinowskiDifference
{
public:

void CsoSupport(const Collider& colliderA, const Collider& colliderB, const Vector3& dir, Vector3& supportA, Vector3& supportB, Vector3& support){
    const Vector3 dirA = colliderA.getRigidBody()->globalToLocalVector(dir);
    const Vector3 dirB = colliderB.getRigidBody()->globalToLocalVector(Vector3() - dir);

    supportA = colliderA.support(dirA);
    supportB = colliderB.support(dirB);

    supportA = colliderA.getRigidBody()->localToGlobalVector(supportA);
    supportB = colliderB.getRigidBody()->localToGlobalVector(supportB);

    support = supportA - supportB; // CSO support point
}

};



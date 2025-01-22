#pragma once

#include "../core/Vector3.h"
#include "../core/RigidBody.h"
#include "../core/Collider.h"

class MinowskiDifference
{
public:

void CsoSupport(const Collider& colliderA, const Collider& colliderB, const Vector3& dir, Vector3& supportA, Vector3& supportB, Vector3& support){
    Vector3 dirA = colliderA.m_body->globalToLocalVector(dir);
    Vector3 dirB = colliderB.m_body->globalToLocalVector(-dir);

    supportA = colliderA.Support(dirA);
    supportB = colliderB.Support(dirB);

    supportA = colliderA.getRigidBody()->localToGlobalVector(supportA);
    supportB = colliderB.getRigidBody()->localToGlobalVector(supportB);

    support = supportA - supportB; // CSO support point

}

};



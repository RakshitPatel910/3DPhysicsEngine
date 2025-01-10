#include<algorithm>

#include "Vector3.h"
#include "RigidBody.h"
#include "Collider.h"
#include "ContactConstraint.cpp"

void calcJacobian(){
    Vector3 contactNormal = contactData->contactNormal;

    Vector3 motionA = contactData.worldContactPointA - colliderA->m_body.getPosition();
    Vector3 motionB = contactData.worldContactPointB - colliderB->m_body.getPosition();

    Vector3 rotMotionA = motionA.cross(contactNormal);
    Vector3 rotMotionB = motionB.cross(contactNormal);

    Eigen::Matrix<float, 1, 3> Ja << -contactNormal.getX(), -contactNormal.getY(), -contactNormal.getZ(), -rotMotionA.getX(), -rotMotionA.getY(), -rotMotionA.getZ();
    Eigen::Matrix<float, 1, 3> Jb << contactNormal.getX(), contactNormal.getY(), contactNormal.getZ(), rotMotionB.getX(), rotMotionB.getY(), rotMotionB.getZ();

    if( colliderA->m_colliderType == ColliderType::STATIC ){
        Ja *= 0.0f;
    }

    if( colliderB->m_colliderType == ColliderType::STATIC ){
        Jb *= 0.0f;
    }

    J << Ja, Jb;

    effectiveMass = J * M_inv * J.transpose();
}

void solveConstraint(float time, std::shared_ptr<ContactConstraint> contact){
    Collider* collA = contact->colliderA;
    Collider* collB = contact->colliderB;

    Vector3 contactNormal contact->contactData->contactNormal;

    Vector3 Va = collA->m_body.getVelocity();
    Vector3 Wa = collA->m_body.getAngularVelocity();
    Vector3 Vb = collB->m_body.getVelocity();
    Vector3 Wb = collB->m_body.getAngularVelocity();

    Vector3 ra = contactData.worldContactPointA - colliderA->m_body.getPosition();
    Vector3 rb = contactData.worldContactPointB - colliderB->m_body.getPosition();

    float JV = (Vb + Wb.cross(rb) - Va - Wa.cross(ra)).dot(contactNormal);

    const float baumgarte_beta = 0.005f;
    float baumgarte_term = (-baumgarte_beta * contact->contactData->penetrationDepth) / time; // or
    // float baumgarte_term = (-baumgarte_beta * JV) / time;

    float oldImpulseSum = contact->contactData->normalImpulseSum;
    float lambda = -(JV + baumgarte_term) / effectiveMass;

    float impulseSum = oldImpulseSum + lambda;
    impulseSum = std::max(0.0f, std::min(impulseSum, FLT_MAX));
    contact->contactData->contactNormal = impulseSum;

    lambda = impulseSum - oldImpulseSum;

    
}
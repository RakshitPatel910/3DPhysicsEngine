#include<algorithm>
#include<Eigen/Dense>

#include "Vector3.h"
#include "RigidBody.h"
#include "Collider.h"
#include "ContactConstraint.h"
#include "EPA.h"

void ContactConstraint::calcJacobian(){
    Vector3 contactNormal = contactData->contactNormal;

    Vector3 motionA = contactData->worldContactPointA - colliderA->m_body->getPosition();
    Vector3 motionB = contactData->worldContactPointB - colliderB->m_body->getPosition();

    Vector3 rotMotionA = motionA.cross(contactNormal);
    Vector3 rotMotionB = motionB.cross(contactNormal);

    Eigen::Matrix<float, 1, 6> Ja;
    Ja << -contactNormal.getX(), -contactNormal.getY(), -contactNormal.getZ(), -rotMotionA.getX(), -rotMotionA.getY(), -rotMotionA.getZ();
    Eigen::Matrix<float, 1, 6> Jb;
    Jb << contactNormal.getX(), contactNormal.getY(), contactNormal.getZ(), rotMotionB.getX(), rotMotionB.getY(), rotMotionB.getZ();

    if( colliderA->m_colliderType == Collider::ColliderType::STATIC ){
        Ja *= 0.0f;
    }

    if( colliderB->m_colliderType == Collider::ColliderType::STATIC ){
        Jb *= 0.0f;
    }

    J << Ja, Jb;

    Minv_Jtr << M_inv * J.transpose();

    effectiveMass = J * Minv_Jtr;
}

// void ContactConstraint::solveConstraint(float time, std::shared_ptr<ContactData> contact){
float ContactConstraint::solveConstraint(float time){
    Collider* collA = colliderA;
    Collider* collB = colliderB;

    // Vector3 contactNormal contact->contactData->contactNormal;
    Vector3 contactNormal = contactData->contactNormal;

    Vector3 Va = collA->m_body->getVelocity();
    Vector3 Wa = collA->m_body->getAngularVelocity();
    Vector3 Vb = collB->m_body->getVelocity();
    Vector3 Wb = collB->m_body->getAngularVelocity();

    Vector3 ra = contactData->worldContactPointA - colliderA->m_body->getPosition();
    Vector3 rb = contactData->worldContactPointB - colliderB->m_body->getPosition();

    float JV = (Vb + Wb.cross(rb) - Va - Wa.cross(ra)).dot(contactNormal);

    const float baumgarte_beta = 0.005f;
    // float baumgarte_term = (-baumgarte_beta * contact->contactData->penetrationDepth) / time; // or
    float baumgarte_term = (-baumgarte_beta * contactData->penetrationDepth) / time; // or
    // float baumgarte_term = (-baumgarte_beta * JV) / time;

    // float oldImpulseSum = contact->contactData->normalImpulseSum;
    float oldImpulseSum = contactData->normalImpulseSum;
    float lambda = -(JV + baumgarte_term) / effectiveMass;

    float impulseSum = oldImpulseSum + lambda;
    impulseSum = std::max(0.0f, std::min(impulseSum, FLT_MAX));
    // contact->contactData->contactNormal = impulseSum;
    // contactData->contactNormal = impulseSum;

    lambda = impulseSum - oldImpulseSum;

    Eigen::Matrix<float, 12, 1> delta_V = Minv_Jtr * lambda; // Catto_A

    Va = Va - Vector3(delta_V(0, 0), delta_V(1, 0), delta_V(2, 0));
    Wa = Wa - Vector3(delta_V(3, 0), delta_V(4, 0), delta_V(5, 0));
    Vb = Vb + Vector3(delta_V(6, 0), delta_V(7, 0), delta_V(8, 0));
    Wb = Wb + Vector3(delta_V(9, 0), delta_V(10, 0), delta_V(11, 0));

    collA->m_body->v = Va;
    collA->m_body->ang_v = Wa;
    collB->m_body->v = Vb;
    collB->m_body->ang_v = Wb;

    return lambda;
}
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

    Vector3 rotMotionA = contactNormal.cross(motionA);
    Vector3 rotMotionB = contactNormal.cross(motionB);

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

    effectiveMass = (J * Minv_Jtr)(0,0);

    // ---- LOGGING BEGIN ----
    std::cout << "---- calcJacobian() ----\n";
    
    std::cout << "Jacobian Ja: [" 
              << Ja(0)<<","<<Ja(1)<<","<<Ja(2)<<","<<Ja(3)<<","<<Ja(4)<<","<<Ja(5)<<"]\n";
    std::cout << "Jacobian Jb: [" 
              << Jb(0)<<","<<Jb(1)<<","<<Jb(2)<<","<<Jb(3)<<","<<Jb(4)<<","<<Jb(5)<<"]\n";
    std::cout << "effectiveMass: " << effectiveMass << "\n";
    // ---- LOGGING END ----
}

// void ContactConstraint::solveConstraint(float time, std::shared_ptr<ContactData> contact){
float ContactConstraint::solveConstraint(float dt) {
    // Relative normal velocity
    Vector3 n = contactData->contactNormal;
    Collider* A = colliderA; Collider* B = colliderB;
    Vector3 Va = A->m_body->v, Wa = A->m_body->ang_v;
    Vector3 Vb = B->m_body->v, Wb = B->m_body->ang_v;
    Vector3 ra = contactData->worldContactPointA - A->m_body->getPosition();
    Vector3 rb = contactData->worldContactPointB - B->m_body->getPosition();
    float JV = (Vb + Wb.cross(rb) - Va - Wa.cross(ra)).dot(n);
    
    // Baumgarte stabilization (push out penetration)
    const float beta = 0.3f;
    float baum = -(beta / dt) * contactData->penetrationDepth;

    // Restitution: apply only if bodies are closing
    const float restitution = 0.1f; // or contactData->restitution
    float rawLambda;
    if (JV < 0.0f) {
        // Incorporate (1+e) factor for bounce
        rawLambda = -((1.0f + restitution) * JV + baum) / effectiveMass;
    } else {
        // Separating or stationary: only correct penetration (no bounce impulse)
        rawLambda = -(JV + baum) / effectiveMass;
    }
    
    // Accumulate and clamp normal impulse (≥ 0)
    float oldSum = contactData->normalImpulseSum;
    float newSum = std::max(0.0f, oldSum + rawLambda);
    float dLambda = newSum - oldSum;
    contactData->normalImpulseSum = newSum;

    // Apply impulse ΔV = M^-1 * J^T * dLambda
    Eigen::Matrix<float,12,1> deltaV = Minv_Jtr * dLambda;
    Va += Vector3(deltaV(0), deltaV(1), deltaV(2));
    Wa += Vector3(deltaV(3), deltaV(4), deltaV(5));
    Vb += Vector3(deltaV(6), deltaV(7), deltaV(8));
    Wb += Vector3(deltaV(9), deltaV(10), deltaV(11));

    A->m_body->v = Va;  A->m_body->ang_v = Wa;
    B->m_body->v = Vb;  B->m_body->ang_v = Wb;

    return dLambda;
}




// ! This one does not work.
// float ContactConstraint::solveConstraint(float time){
    
//     if (std::abs(effectiveMass) < 1e-6f) {
//         return 0.0f; // Skip this constraint
//     }

//     std::cout << "pen dep : " << contactData->penetrationDepth << '\n';

//     const float penetrationThreshold = 1e-4f;
//     if(contactData->penetrationDepth <= penetrationThreshold ||
//        (fabs(contactData->contactNormal.getX()) < 1e-6f &&
//         fabs(contactData->contactNormal.getY()) < 1e-6f &&
//         fabs(contactData->contactNormal.getZ()) < 1e-6f))
//     {
//         return 0.0f;
//     }

//     Collider* collA = colliderA;
//     Collider* collB = colliderB;

//     Vector3 Va = collA->m_body->getVelocity();
//     Vector3 Wa = collA->m_body->getAngularVelocity();
//     Vector3 Vb = collB->m_body->getVelocity();
//     Vector3 Wb = collB->m_body->getAngularVelocity();

//     Vector3 ra = contactData->worldContactPointA - colliderA->m_body->getPosition();
//     Vector3 rb = contactData->worldContactPointB - colliderB->m_body->getPosition();

//     Vector3 vDelta = Vb + Wb.cross(rb) - Va - Wa.cross(ra);
//     float dotDN = vDelta.dot(contactData->contactNormal);

//     float biasImpulse = 0.f;
//     if (contactData->penetrationDepth > 0.005f) //slop = 0.005
//         biasImpulse = (0.1f / time) * std::max(0.f, contactData->penetrationDepth - 0.005f);
//     float b = biasImpulse;

//     std::cout << "EffectiveMass: " << effectiveMass << ", biasImpulse: " << biasImpulse << "\n";


//     float lambda = -(dotDN - b) / effectiveMass;
//     float newImpulse = std::max(contactData->normalImpulseSum + lambda, 0.0f);
//     lambda = newImpulse - contactData->normalImpulseSum;
//     // contactData->normalImpulseSum += newImpulse;
//     contactData->normalImpulseSum = newImpulse;

//     Eigen::Matrix<float, 12, 1> delta_V = Minv_Jtr * lambda; // Catto_A

//     Va = Va - Vector3(delta_V(0, 0), delta_V(1, 0), delta_V(2, 0));
//     Wa = Wa - Vector3(delta_V(3, 0), delta_V(4, 0), delta_V(5, 0));
//     Vb = Vb + Vector3(delta_V(6, 0), delta_V(7, 0), delta_V(8, 0));
//     Wb = Wb + Vector3(delta_V(9, 0), delta_V(10, 0), delta_V(11, 0));

//     collA->m_body->v = Va;
//     collA->m_body->ang_v = Wa;
//     collB->m_body->v = Vb;
//     collB->m_body->ang_v = Wb;

//     return lambda;
// }
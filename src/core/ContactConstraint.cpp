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
    std::cout << "--- Minv_Jtr ---\n";
    std::cout << Minv_Jtr.transpose() << "\n\n";


    std::cout << "---- calcJacobian() ----\n";
    
    std::cout << "Jacobian Ja: [" 
              << Ja(0)<<","<<Ja(1)<<","<<Ja(2)<<","<<Ja(3)<<","<<Ja(4)<<","<<Ja(5)<<"]\n";
    std::cout << "Jacobian Jb: [" 
              << Jb(0)<<","<<Jb(1)<<","<<Jb(2)<<","<<Jb(3)<<","<<Jb(4)<<","<<Jb(5)<<"]\n";
    std::cout << "effectiveMass: " << effectiveMass << "\n";
    // ---- LOGGING END ----
}

// void ContactConstraint::solveConstraint(float time, std::shared_ptr<ContactData> contact){
float ContactConstraint::solveConstraint(float /*dt*/, int iter) {
    // 1) compute relative normal velocity JV
    Vector3 n  = contactData->contactNormal;
    auto A = colliderA->m_body.get();
    auto B = colliderB->m_body.get();

    Vector3 Va = A->v, Wa = A->ang_v;
    Vector3 Vb = B->v, Wb = B->ang_v;
    Vector3 ra = contactData->worldContactPointA - A->getPosition();
    Vector3 rb = contactData->worldContactPointB - B->getPosition();

    float JV = (Vb + Wb.cross(rb) - Va - Wa.cross(ra)).dot(n);

    // 2) fetch the ONE‑TIME bias & restitution we precomputed at constraint‑build
    float baum = contactData->biasTerm / float(16);
    float rest = contactData->restitutionTerm;

    // 3) compute impulse (no extra (1+e) or baum here!)
    float rawLambda = -(JV + baum + rest) / effectiveMass;

    // 4) accumulate & clamp  
    float oldSum = contactData->normalImpulseSum;
    float newSum = std::max(0.0f, oldSum + rawLambda);
    float dLambda = newSum - oldSum;
    contactData->normalImpulseSum = newSum;

    // 5) apply Δv = M⁻¹·Jᵀ·Δλ
    Eigen::Matrix<float,12,1> Δv = Minv_Jtr * dLambda;
    Va += Vector3(Δv(0),  Δv(1),  Δv(2));
    Wa += Vector3(Δv(3),  Δv(4),  Δv(5));
    Vb += Vector3(Δv(6),  Δv(7),  Δv(8));
    Wb += Vector3(Δv(9),  Δv(10), Δv(11));

    A->v     = Va;
    A->ang_v = Wa;
    B->v     = Vb;
    B->ang_v = Wb;

    // 6) debug logging
    std::cout 
      << "[Contact " << this 
      << " | iter "    << iter 
      << "] JV="       << JV 
      << ", bias="    << baum
      << ", rest="    << rest 
      << ", rawΛ="    << rawLambda 
      << ", dΛ="      << dLambda 
      << ", sum="     << newSum << "\n";
    std::cout << "  Va: "; Va.printV();
    std::cout << "  Vb: "; Vb.printV();

    return dLambda;
}
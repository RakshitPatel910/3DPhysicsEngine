#include "RigidBody.h"
#include "Collider.h"
#include "Constraint.h"

void Constraint::calcInverseMassMatrix() {
    Eigen::Matrix3f invMassMatA = Eigen::Matrix3f::Zero();
    if (colliderA->m_colliderType == Collider::ColliderType::DYNAMIC)
        invMassMatA = Eigen::Matrix3f::Identity() * (1.0f / colliderA->m_mass);


    Eigen::Matrix3f inverseInertiaTensorA = Eigen::Matrix3f::Zero();
    if (colliderA->m_colliderType == Collider::ColliderType::DYNAMIC)
        inverseInertiaTensorA = colliderA->m_body->getInverseInertiaTensor().toEigenMatrix3f();


    Eigen::Matrix3f invMassMatB = Eigen::Matrix3f::Zero();
    if (colliderB->m_colliderType == Collider::ColliderType::DYNAMIC)
        invMassMatB = Eigen::Matrix3f::Identity() * (1.0f / colliderB->m_mass);

    Eigen::Matrix3f inverseInertiaTensorB = Eigen::Matrix3f::Zero();
    if (colliderB->m_colliderType == Collider::ColliderType::DYNAMIC)
        inverseInertiaTensorB = colliderB->m_body->getInverseInertiaTensor().toEigenMatrix3f();

    M_inv.setZero(); // clear to start clean

    // A's inverse mass & inertia
    M_inv.block<3,3>(0,0)   = invMassMatA;
    M_inv.block<3,3>(3,3)   = inverseInertiaTensorA;

    // B's inverse mass & inertia
    M_inv.block<3,3>(6,6)   = invMassMatB;
    M_inv.block<3,3>(9,9)   = inverseInertiaTensorB;

    std::cout << "--- Inverse Mass Matrix M_inv ---\n";
    std::cout << M_inv << "\n\n";
}
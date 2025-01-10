#include "RigidBody.h"
#include "Collider.h"
#include "Constraint.h"

void Constraint::calcInverseMassMatrix() {
    Eigen::Matrix3f massMatA;
    massMatA.setIdentity();
    massMatA = massMatA * colliderA->m_mass;
    Eigen::Matrix3f invMassMatA = massMatA.inverse();

    Eigen::Matrix3f inverseInertiaTensorA = colliderA->m_body->getInverseInertiaTensor().toEigenMatrix3f();

    Eigen::Matrix3f massMatB;
    massMatB.setIdentity();
    massMatB = massMatB * colliderB->m_mass;
    Eigen::Matrix3f invMassMatB = massMatB.inverse();

    Eigen::Matrix3f inverseInertiaTensorB = colliderB->m_body->getInverseInertiaTensor().toEigenMatrix3f();

    M_inv << invMassMatA, Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(),
             Eigen::Matrix3f::Zero(), inverseInertiaTensorA, Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(),
             Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), invMassMatB, Eigen::Matrix3f::Zero(),
             Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), inverseInertiaTensorB;

}
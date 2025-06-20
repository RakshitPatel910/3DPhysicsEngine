#pragma once

#include "Vector3.h"
#include "Matrix4.h"
#include "RigidBody.h"
#include "AABB.h"
#include "./shape/Shape.h"

#include <memory>

class AABB;

class Collider
{
public:
    enum ColliderType
	{
		STATIC,
		DYNAMIC
	};

    float m_mass;
    Matrix4 m_localInertiaTensor;
    Vector3 m_localcentroid;
    // RigidBody* m_body;
    std::shared_ptr<RigidBody> m_body;
    Shape* m_shape;
    ColliderType m_colliderType;
    AABB m_aabb;


    Collider(
        float mass, 
        const Matrix4& inertiaTensor, 
        const Vector3& centroid, 
        std::shared_ptr<RigidBody> body,
        Shape* shape,
        ColliderType colliderType = ColliderType::DYNAMIC
    ) : 
        m_mass(mass), 
        m_localInertiaTensor(inertiaTensor), 
        m_localcentroid(centroid), 
        m_body(body), 
        m_shape(shape), 
        m_colliderType(colliderType) 
    {
        // Generate an AABB for this Collider
        m_aabb.collider = this;
    }


    // RigidBody* getRigidBody() const {
    std::shared_ptr<RigidBody> getRigidBody() const {
        return m_body;
    }

    

    Vector3 Support(const Vector3& dir) const {
        // return m_shape->Support(dir);

        Vector3 localDir = m_body->globalToLocalVector(dir);

        // std::cout << "local dir: "; localDir.printV();

        Vector3 support = m_shape->Support(localDir) + m_body->getPosition();

        // std::cout << "local support:"; support.printV();

        support = m_body->localToGlobalVector(support);

        // std::cout << "local support:"; support.printV();

        return support;
    }
};


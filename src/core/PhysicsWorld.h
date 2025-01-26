#pragma once

#include <vector>
#include <memory>
#include <map>

#include "RigidBody.h"
#include "Collider.h"
#include "Broadphase.h"
#include "AABBTree.h"
#include "Simplex.h"
#include "GJK.h"
#include "EPA.h"
#include "Manifold.h"
#include "ContactConstraint.h"

class PhysicsWorld
{
public:
    std::vector<std::shared_ptr<RigidBody>> rigidBodies;
    std::map<RigidBody*, Collider*> colliderMap;
    std::map<std::pair<Collider*, Collider*>, Manifold> manifolds;
    std::vector<std::unique_ptr<Constraint>> constraints;

    float dt = 0.016f // 60 fps

    AABBTree broadphase;
    GJK gjk;
    EPA epa;
    

    void addRigidBody(std::shared_ptr<RigidBody> rbody){
        rigidBodies.push_back(rbody);
    }

    void simulate(){
        // Integrate forces for all rigid bodies
        // for( RigidBody& rbody : rigidBodies ){
        //     // rbody->integratePhysics(dt);
        //     rbody->applyForce(); // if any
        // }

        // Broadphase Collision Detection using AABBTree
        for( RigidBody& rbody : rigidBodies ){
            broadphase.Add( colliderMap[rbody]->m_aabb );
        }

        broadphase.Update();

        ColliderPairList& collidingPairs = broadphase.ComputeCollidingPairs();

        // Narrowphase Collision Detection on colliding pairs
        for( const auto& colPair : collidingPairs ){
            Collider* colliderA = pair.first;
            Collider* colliderB = pair.second;

            Simplex simplex;

            // GJK for narrowphase collision detection  
            bool isColliding = gjk.isIntersecting(simplex, *colliderA, *colliderB);

            if( isColliding ){ // if colliding ===> use EPA to generate contact point
                ContactData contact = epa.RunEPA(simplex, *colliderA, *colliderB);

                auto manifoldKey = std::make_pair(colliderA, colliderB)
                if( manifolds.find(manifoldKey) == manifolds.end() ) {
                    manifolds[manifoldKey] = Manifold(colliderA, colliderB);

                    // manifolds[manifoldKey].addContactPoint(contact);
                }
                // else {
                //     manifolds[manifoldKey].updateManifold();
                //     manifolds[manifoldKey].addContactPoint(contact);
                // }

                Manifold& manifold = manifolds[manifoldKey];

                manifold.updateManifold();
                manifold.addContactPoint(contact);

                // Create ContactConstraints for all collisions
                auto constraint = std::unique_ptr<ContactConstraint>(colliderA, colliderB);
                constraint->contactData = &contact;

                constraint->calcJacobian();

                constraints.push_back(std::move(constraint));
            }
        }

        // Collision Resolution ===> i.e. solve constraints
        for( auto& constraint : constraints ) {
            constraint->solveConstraint(dt);
        }

        // Velocity integration 
        for( auto& rbody : rigidBodies ) {
            rbody->integratePhysics(dt);
        }
    }
};


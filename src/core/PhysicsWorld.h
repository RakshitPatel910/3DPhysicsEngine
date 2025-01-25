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

    float dt = 0.016f // 60 fps

    AABBTree broadphase;
    GJK gjk;
    EPA epa;
    

    void addRigidBody(std::shared_ptr<RigidBody> rbody){
        rigidBodies.push_back(rbody);
    }

    void simulate(){
        // Integrate forces for all rigid bodies
        for( RigidBody& rbody : rigidBodies ){
            rbody.integratePhysics(dt);
        }

        // Broadphase Collision Detection using AABBTree
        for( RigidBody& rbody : rigidBodies ){
            broadphase.Add( colliderMap[&rbody]->m_aabb );
        }

        broadphase.Update();

        ColliderPairList& collidingPairs = broadphase.ComputeCollidingPairs();

        // Narrowphase Collision Detection on colliding pairs
        for( auto& colPair : collidingPairs ){
            Simplex simplex;

            // GJK for narrowphase collision detection  
            bool isColliding = gjk.isIntersecting(simplex, *colPair.first, *colPair.second);

            if( isColliding ){ // if colliding ===> use EPA to generate contact point
                ContactData contact = epa.RunEPA(simplex, *colPair.first, *colPair.second);

                if( manifolds.find({colPair.first, colPair.second}) == manifolds.end() ) {
                    manifolds[{colPair.first, colPair.second}] = Manifold(colPair.first, colPair.second);

                    manifolds[{colPair.first, colPair.second}].addContactPoint(contact);
                }
                else {
                    manifolds[{colPair.first, colPair.second}].updateManifold();
                    manifolds[{colPair.first, colPair.second}].addContactPoint(contact);
                }
            }
        }

        // Collision Resolution 
        
    }
};


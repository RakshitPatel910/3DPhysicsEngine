#pragma once

#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <unordered_map>
#include <Eigen/Dense>

#include "RigidBody.h"
#include "Collider.h"
#include "Broadphase.h"
#include "AABBTree.h"
#include "Simplex.h"
#include "GJK.h"
#include "EPA.h"
#include "Manifold.h"
#include "ContactConstraint.h"
#include "../rendering/Renderer.h"

class PhysicsWorld
{
public:
    std::vector<std::shared_ptr<RigidBody>> rigidBodies;
    // std::map<RigidBody*, Collider*> colliderMap;
    std::map<std::shared_ptr<RigidBody>, std::shared_ptr<Collider>> colliderMap;
    std::map<std::pair<Collider*, Collider*>, Manifold> manifolds;
    std::vector<std::unique_ptr<ContactConstraint>> constraints;

    float dt = 0.016f; // 60 fps

    AABBTree broadphase;
    GJK gjk;
    EPA epa;
    
    Renderer* renderer;

    Vector3 gravity = Vector3(0.0f, -0.01f, 0.0f); // Downward along Y-axis

    PhysicsWorld(Renderer* re) : renderer(re) {}

    void addRigidBody(std::shared_ptr<RigidBody> rbody, std::shared_ptr<Collider> coll){
        rigidBodies.push_back(rbody);
        colliderMap[rbody] = coll;

        broadphase.Add( &(coll->m_aabb) );
    }

    // void SolveConstraints(){
    //     if( constraints.size() == 0 ) return;
        
    //     std::unordered_map<Collider*,  Eigen::Matrix<float, 6, 1>> delta_V_cache;
    //     // delta_V_cache.reserve(colliderMap.size());

    //     for( auto cm : colliderMap ){
    //         if( delta_V_cache.find(cm.second) != delta_V_cache.end() ){
    //             delta_V_cache[cm.second] = Eigen::Matrix<float, 6, 1>();
    //             delta_V_cache[cm.second].setZero();
    //         }
    //     }

        
    // }

    // ! No problem in broadphase(might me small prblms) and gjk epa, manifolds and constraint solving does have problem 
    void simulate(){
        using Clock = std::chrono::high_resolution_clock;
        auto totalStart = Clock::now();

        // Integrate forces for all rigid bodies
        // for( RigidBody& rbody : rigidBodies ){
        //     // rbody->integratePhysics(dt);
        //     rbody->applyForce(); // if any
        // }

        // Broadphase Collision Detection using AABBTree
        // for( RigidBody& rbody : rigidBodies ){
        
        auto start = Clock::now();
        // broadphase.ClearTree();
        // for( std::shared_ptr<RigidBody>& rbody : rigidBodies ){
        //     broadphase.Add( &(colliderMap[rbody]->m_aabb) );
        // }

        broadphase.Update();

        ColliderPairList& collidingPairs = broadphase.ComputeCollidingPairs();

        // for( auto& [cola, colb] : collidingPairs ) {
        //     std::cout << cola->m_body->m_name << " " << colb->m_body->m_name << '\n'; 
        // }

        auto end = Clock::now();
        // std::cout << "Broadphase: " 
        //         << std::chrono::duration<double, std::milli>(end - start).count() 
        //         << " ms\n";


        start = Clock::now();
        // Narrowphase Collision Detection on colliding pairs
        for( auto& colPair : collidingPairs ){
            Collider* colliderA = colPair.first;
            Collider* colliderB = colPair.second;

            Simplex simplex;

            // GJK for narrowphase collision detection  
            bool isColliding = gjk.isIntersecting(simplex, *colliderA, *colliderB);
            // bool isColliding = false;

            // std::cout << "simplex : "; simplex.printSimplex();
            
            // std::cout << "----------" << isColliding << "-----------" << '\n';

            // colliderA->m_body->getPosition().printV();
            // colliderB->m_body->getPosition().printV();
            if( isColliding ){ // if colliding ===> use EPA to generate contact point
                // simplex.printSimplex();

                ContactData contact = epa.RunEPA(simplex, *colliderA, *colliderB);

                if(contact.penetrationDepth < 0.0001f) continue;
                std::cout << "Contact Normal: "; contact.contactNormal.printV();
                std::cout << "Penetration Depth: " << contact.penetrationDepth << "\n";


                auto manifoldKey = std::make_pair(colliderA, colliderB);
                if( manifolds.find(manifoldKey) == manifolds.end() ) {
                    manifolds[manifoldKey] = Manifold(colliderA, colliderB);
                    // manifolds.emplace(manifoldKey, colliderA, colliderB);

                    // manifolds[manifoldKey].addContactPoint(contact);
                }
                else {
                    manifolds[manifoldKey].updateManifold();
                    manifolds[manifoldKey].addContactPoint(contact);
                }

                Manifold& manifold = manifolds[manifoldKey];

                manifold.updateManifold();
                manifold.addContactPoint(contact);

                // Create ContactConstraints for all collisions
                auto constraint = std::make_unique<ContactConstraint>(colliderA, colliderB);
                // constraint->contactData = &contact;
                constraint->contactData = std::make_unique<ContactData>(contact);

                constraint->calcJacobian();

                constraints.push_back(std::move(constraint));
            }
        }
        end = Clock::now();
        // std::cout << "Narrowphase (GJK + EPA): " 
        //           << std::chrono::duration<double, std::milli>(end - start).count() 
        //           << " ms\n";

        // Collision Resolution ===> i.e. solve constraints
        // std:: cout << "len : " << constraints.size() << "\n";
        for( auto& constraint : constraints ) {
            // constraint->contactData->contactNormal.printV();
            constraint->calcJacobian();
            constraint->solveConstraint(dt);
        }
        constraints.clear();

        // for (int i = 0; i < 10; ++i) {
        //     for (auto& constraint : constraints) {
        //         constraint->solveConstraint(dt);
        //     }
        // }
        // constraints.clear();

        start = Clock::now();
        // Velocity integration 
        // for( auto& rbody : rigidBodies ) {
        //     if (rbody->mass > 0.0f) { // Only apply to dynamic bodies
        //         rbody->applyForce(gravity * rbody->mass, Vector3(0,0,0));
        //     }

        //     rbody->integratePhysics(dt);
            
        //     // std::cout << "pos out :"; rbody->pos.printV();
        // }
        for( std::shared_ptr<RigidBody>& rbody : rigidBodies ) {
            if (colliderMap[rbody]->m_colliderType == Collider::ColliderType::DYNAMIC) { // Only apply to dynamic bodies
                rbody->applyForce(gravity * rbody->mass, rbody->getPosition());
            }

            rbody->integratePhysics(dt);
            
            // std::cout << "pos out :"; rbody->pos.printV();
        }
        end = Clock::now();
        // std::cout << "Velocity Integration: " 
        //         << std::chrono::duration<double, std::milli>(end - start).count() 
        //         << " ms\n";

        start = Clock::now();
        for( std::shared_ptr<RigidBody>& rbody : rigidBodies ) {
            
            renderer->submitMesh(*(colliderMap[rbody]->m_shape->mesh), rbody->getTransformMatrix().toGlm());
            // colliderMap[rbody]->m_shape->mesh->drawWireframe();
        }
        end = Clock::now();
        // std::cout << "Rendering Submission: " 
        //         << std::chrono::duration<double, std::milli>(end - start).count() 
        //         << " ms\n";


        auto totalEnd = Clock::now();
        // std::cout << "Total Simulation Time: " 
        //             << std::chrono::duration<double, std::milli>(totalEnd - totalStart).count() 
        //             << " ms\n";
    }
};


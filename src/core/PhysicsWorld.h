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
    void simulate() {
        // 1) Broadphase
        broadphase.Update();
        auto& pairs = broadphase.ComputeCollidingPairs();

        // 2) Prune stale manifold contacts before detecting new ones
        for (auto& kv : manifolds) {
            kv.second.updateManifold();
        }

        // 3) Build contact constraints for _all_ colliding pairs
        constraints.clear();
        for (auto& pr : pairs) {
            Collider* A = pr.first;
            Collider* B = pr.second;

            Simplex simplex;
            if (!gjk.isIntersecting(simplex, *A, *B)) continue;

            ContactData cd = epa.RunEPA(simplex, *A, *B);
            if (cd.penetrationDepth < 1e-4f) continue;


            // ——— PRECOMPUTE BIAS & RESTITUTION ONCE PER FRAME ———
            // 1) relative normal velocity JV0 at the contact
            Vector3 n  = cd.contactNormal;
            Vector3 Va = A->m_body->v,  Wa = A->m_body->ang_v;
            Vector3 Vb = B->m_body->v,  Wb = B->m_body->ang_v;
            Vector3 ra = cd.worldContactPointA - A->m_body->getPosition();
            Vector3 rb = cd.worldContactPointB - B->m_body->getPosition();
            float JV0 = (Vb + Wb.cross(rb) - Va - Wa.cross(ra)).dot(n);

            // 2) Baumgarte bias (only correct penetration beyond a small slop)
            const float beta = 0.2f;
            const float slop = 0.001f;
            float pen   = cd.penetrationDepth;
            float depth = std::max(0.0f, pen - slop);
            cd.biasTerm = -(beta / dt) * depth;

            // 3) Restitution (only if bodies are closing in)
            const float e = 0.1f;
            cd.restitutionTerm = (JV0 < 0.0f) ? (-e * JV0) : 0.0f;


            auto key = std::make_pair(A, B);
            auto it  = manifolds.find(key);
            if (it == manifolds.end())
                it = manifolds.emplace(key, Manifold(A,B)).first;
            Manifold& m = it->second;

            // Add or update the persistent contact; get its pointer
            ContactData* persistent = m.addContactPoint(cd);


            std::cout << "Creating ContactConstraint between A=" << A << " and B=" << B; 
            std::cout << " at point A = "; cd.worldContactPointA.printV(); 
            std::cout << ", point B = "; cd.worldContactPointB.printV(); 
            std::cout << ", normal = "; cd.contactNormal.printV(); 
            std::cout << ", pen = " << cd.penetrationDepth << '\n';
            std::cout << "Pushing constraint #" << constraints.size() + 1 << " for A=" << A << ", B=" << B << "\n";
            std::cout << "Total contact constraints this frame: " << constraints.size() << "\n\n";



            // Create a solver constraint pointing into that persistent data
            auto c = std::make_unique<ContactConstraint>(A, B, persistent);
            constraints.push_back(std::move(c));
        }

        // 4) Warm‑start: re‑apply each contact’s stored normal impulse
        for (auto& c : constraints) {
            c->calcJacobian();
            float λ0 = c->contactData->normalImpulseSum;
            Eigen::Matrix<float,12,1> dV = c->Minv_Jtr * λ0;
            
            auto bodyA = c->colliderA->m_body.get();
            auto bodyB = c->colliderB->m_body.get();
            
            bodyA->v     += Vector3(dV(0), dV(1), dV(2));
            bodyA->ang_v += Vector3(dV(3), dV(4), dV(5));
            bodyB->v     += Vector3(dV(6), dV(7), dV(8));
            bodyB->ang_v += Vector3(dV(9), dV(10),dV(11));
            
            // c->contactData->normalImpulseSum = 0.0f;
        }

        // 5) Sequential‑impulse solve (Gauss–Seidel style)
        const int solverIters = 16;
        for (int iter = 0; iter < solverIters; ++iter) {
            for (auto& c : constraints) {
                // c->calcJacobian();
                c->solveConstraint(dt, iter);
            }
        }

        // 6) Integrate velocities & positions
        for (auto& rbptr : rigidBodies) {
            RigidBody* rb = rbptr.get();
            // apply gravity
            if (colliderMap[rbptr]->m_colliderType == Collider::ColliderType::DYNAMIC)
                rb->applyForce(gravity * rb->mass, rb->getPosition());
            rb->integratePhysics(dt);
        }

        // 7) Submit to renderer
        for (auto& rbptr : rigidBodies) {
            renderer->submitMesh(
                *(colliderMap[rbptr]->m_shape->mesh),
                rbptr->getTransformMatrix().toGlm()
            );
        }
    }

};


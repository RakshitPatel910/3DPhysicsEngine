#pragma once

#include<list>
#include<vector>

#include "Collider.h"
#include "AABB.h"

typedef std::pair<Collider *, Collider *> ColliderPair;
typedef std::list<ColliderPair> ColliderPairList;
typedef std::vector<Collider *> ColliderList;
typedef std::vector<AABB *> AABBList;

class Broadphase
{
private:

public:
    virtual void Add(AABB* aabb) = 0;

    virtual void Update() = 0;

    virtual ColliderPairList& ComputeCollidingPairs() = 0;

    virtual void Query(const AABB* aabb, const ColliderList& output) const = 0;

    // Pick and Ray will be done later
};

class NSquared : public Broadphase
{
private:
    AABBList m_aabbs;
    ColliderPairList m_collidingPairs;

public:
    virtual void Add(AABB* aabb){
        m_aabbs.push_back(aabb);
    }

    virtual void Update(){}

    virtual ColliderPairList& ComputeCollidingPairs();
    virtual Query(const AABB* aabb, const ColliderList& output) const;

    // Pick and Ray will be done later
}

const ColliderPairList& NSquared::ComputeCollidingPairs(){
    m_collidingPairs.clear();

    auto e = m_aabbs.end();
    for( auto i = m_aabbs.begin(); i != e; i++ ){
        AABB* A = *i;
        Collider* colliderA = A->getCollider();
        RigidBody* rigidBodyA = colliderA->getRigidBody();

        auto s = i;
        for( auto j = ++s; j != e; j++ ){
            AABB* B = *j;
            Collider* colliderB = B->getCollider();
            RigidBody* rigidBodyB = colliderB->getRigidBody();

            if( rigidBodyA == rigidBodyB ) continue;

            if( A->intersects(B) ){
                m_collidingPairs.push_back(std::make_pair(A->collider, B->collider));
            }
        }
    }

    return m_collidingPairs;
}

void NSquared::Query(const AABB* aabb, ColliderList& output) const {
    for( auto &aabb_i : m_aabbs ){
        if( aabb_i->intersects(aabb) ){
            output.push_back(aabb_i->getCollider());
        }
    }
}
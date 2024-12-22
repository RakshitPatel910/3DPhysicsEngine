#pragma once

#include<list>
#include<vector>

#include "Collider.h"
#include "AABB.h"

typedef std::pair<Collider *, Collider *> ColliderPair;
typedef std::list<ColliderPair> ColliderPairList;
typedef std::vector<Collider *> ColliderList;

class Broadphase
{
private:

public:
    virtual void Add(AABB* aabb) = 0;

    virtual void Update() = 0;

    virtual ColliderPairList& CoputeCollidingPairs() = 0;

    virtual const Query(const AABB* aabb, const ColliderList& output) = 0;

    // Pick and Ray will be done later
};


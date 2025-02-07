#pragma once

#include<iostream>

#include "Vector3.h"
// #include "Collider.h"

class Node;
class Collider;

class AABB {
public:
    Vector3 minExt;
    Vector3 maxExt;

    Collider* collider;
    
    Node* nodeData;

    AABB(const Vector3& minExt = Vector3(), const Vector3& maxExt = Vector3());

    static AABB fromHalfCentralExtents(const Vector3& center, const Vector3& halfExt);
    static AABB merge(const AABB& a, const AABB& b);

    Vector3 getCenter() const;
    Vector3 getHalfExtent() const;
    Collider* getCollider() const;
    void expandTofit(const AABB& other);
    bool intersects(const AABB& other);
    bool contains(const AABB& other);
    float getVolume() const;
};

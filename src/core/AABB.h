#pragma once

#include "Vector3.h"
#include "Collider.h"
#include "Node.h"

class AABB
{
public:
    Vector3 minExt;
    Vector3 maxExt;
    Collider* collider;
    Node* nodeData;

    AABB(const Vector3& minExt = Vector3(), const Vector3& maxExt = Vector3()) : minExt(minExt), maxExt(maxExt) {}

    static AABB fromHalfCentralExtents(const Vector3& center, const Vector3& halfExt){
        return AABB(center - halfExt, center + halfExt);
    }

    Vector3 getCenter() const {
        return (minExt + maxExt) * 0.5f; 
    }

    Vector3 getHalfExtent() const {
        return (maxExt - minExt) * 0.5f;
    }

    Collider* getCollider() const {
        return collider;
    }

    void expandTofit(const AABB& other){
        minExt = std::min(minExt, other.minExt);
        maxExt = std::max(maxExt, other.maxExt);
    }

    bool intersects(const AABB& other){
        return (minExt.getX() <= other.maxExt.getX() && maxExt.getX() >= other.minExt.getX()) &&
               (minExt.getY() <= other.maxExt.getY() && maxExt.getY() >= other.minExt.getY()) &&
               (minExt.getZ() <= other.maxExt.getZ() && maxExt.getZ() >= other.minExt.getZ());
    }

    bool contains(const Vector3& pt){
        return (minExt.getX() <= pt.getX() && maxExt.getX() >= pt.getX()) &&
               (minExt.getY() <= pt.getY() && maxExt.getY() >= pt.getY()) &&
               (minExt.getZ() <= pt.getZ() && maxExt.getZ() >= pt.getZ());
    }

    float getVolume() const {
        Vector3 s = maxExt - minExt;

        return s.getX() * s.getY() * s.getZ();
    }

    static AABB merge(const AABB& a, const AABB& b){
        return AABB(std::min(a.minExt, b.minExt), std::max(a.maxExt, b.maxExt));
    }
};


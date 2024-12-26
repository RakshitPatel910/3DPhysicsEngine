#include "AABB.h"
#include "Node.h"

AABB::AABB(const Vector3& minExt, const Vector3& maxExt)
    : minExt(minExt),
      maxExt(maxExt), collider(nullptr),
      nodeData(nullptr) 
    {}

AABB AABB::fromHalfCentralExtents(const Vector3& center, const Vector3& halfExt) {
    return AABB(center - halfExt, center + halfExt);
}

AABB AABB::merge(const AABB& a, const AABB& b) {
    return AABB(std::min(a.minExt, b.minExt), std::max(a.maxExt, b.maxExt));
}

Vector3 AABB::getCenter() const {
    return (minExt + maxExt) * 0.5f;
}

Vector3 AABB::getHalfExtent() const {
    return (maxExt - minExt) * 0.5f;
}

Collider* AABB::getCollider() const {
    return collider;
}

void AABB::expandTofit(const AABB& other) {
    minExt = std::min(minExt, other.minExt);
    maxExt = std::max(maxExt, other.maxExt);
}

bool AABB::intersects(const AABB& other) {
    return (minExt.getX() <= other.maxExt.getX() && maxExt.getX() >= other.minExt.getX()) &&
           (minExt.getY() <= other.maxExt.getY() && maxExt.getY() >= other.minExt.getY()) &&
           (minExt.getZ() <= other.maxExt.getZ() && maxExt.getZ() >= other.minExt.getZ());
}

bool AABB::contains(const AABB& other) {
    return (minExt.getX() <= other.minExt.getX() && maxExt.getX() >= other.maxExt.getX()) &&
           (minExt.getY() <= other.minExt.getY() && maxExt.getY() >= other.maxExt.getY()) &&
           (minExt.getZ() <= other.minExt.getZ() && maxExt.getZ() >= other.maxExt.getZ());
}

float AABB::getVolume() const {
    Vector3 s = maxExt - minExt;
    return s.getX() * s.getY() * s.getZ();
}

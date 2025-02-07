#include "Shape.h"

#include <vector>

Sphere::Sphere(const Vector3& center, const float radius) 
    : m_center(center), m_radius(radius) {}

// Sphere::createMesh(float r, unsigned int slices, unsigned int stacks){
    
//     for(unsigned int i = 0; i <= stacks; i++){
//         float stackAngle = glm::pi<float>() * float(i) / float(stacks);
//     }
// }

Vector3 Sphere::Support(const Vector3& dir) const {
    return m_center + dir.normalized() * m_radius;
}

Cube::Cube(const Vector3& center, const Vector3& halfext) 
    : m_center(center), m_halfext(halfext) {}

Vector3 Cube::Support(const Vector3& dir) const {
    Vector3 vec = Vector3(
        m_halfext.getX() * (dir.getX() >= 0 ? 1 : -1),
        m_halfext.getY() * (dir.getY() >= 0 ? 1 : -1),
        m_halfext.getZ() * (dir.getZ() >= 0 ? 1 : -1)
    );

    return m_center + vec;
}

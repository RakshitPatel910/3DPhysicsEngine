#pragma once

#include "../Vector3.h"
#include "../Mesh.h"

class Shape
{
private:
    Mesh* mesh;
    
public:
    virtual ~Shape() {}
    virtual Vector3 Support(const Vector3& dir) const = 0;
};

class Sphere : public Shape 
{
public:
    Vector3 m_center;
    float m_radius;

    Sphere(const Vector3& center, const float radius) : m_center(center), m_radius(radius) {}

    virtual Vector3 Support(const Vector3& dir) const override {
        return m_center + dir.normalized() * m_radius;
    }
};

class Cube : public Shape
{
public:
    Vector3 m_center;
    Vector3 m_halfext;

    Cube(const Vector3& center, const Vector3& halfext) : m_center(center), m_halfext(halfext) {}

    virtual Vector3 Support(const Vector3& dir) const override {
        Vector3 vec = Vector3(
            m_halfext.getX() * (dir.getX() >= 0 ? 1 : -1),
            m_halfext.getY() * (dir.getY() >= 0 ? 1 : -1),
            m_halfext.getZ() * (dir.getZ() >= 0 ? 1 : -1)
        );

        return m_center + vec;
    }
};

class MeshShape : public Shape
{

};
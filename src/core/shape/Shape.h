#pragma once

#include "../Vector3.h"
#include "../Matrix4.h"
#include "../Mesh.h"

class Shape
{
protected:

public:
const Mesh* mesh;
Matrix4 inertiaTensor;
    Vector3 scaleFactor;

    virtual ~Shape() {}
    virtual Vector3 Support(const Vector3& dir) const = 0;
    virtual void calculateInertiaTensor(float mass) = 0;
    virtual Matrix4 getInertiaTensor() const = 0;
};

class Sphere : public Shape 
{
public:
    // Vector3 m_center;
    float m_radius;

    Sphere(const float radius);

    virtual Vector3 Support(const Vector3& dir) const override;
    virtual void calculateInertiaTensor(float mass) override;
    virtual Matrix4 getInertiaTensor() const override;

private:
    // Mesh* createMesh(float r, unsigned int slices, unsigned int stacks);
};

class Cube : public Shape
{
public:
    // Vector3 m_center;
    Vector3 m_halfext;

    Cube(const Vector3& halfext);

    virtual Vector3 Support(const Vector3& dir) const override;
    virtual void calculateInertiaTensor(float mass) override;
    virtual Matrix4 getInertiaTensor() const override;
};

// class MeshShape : public Shape
// {
//     // Empty or any specific implementation
// };


// #include "Mesh.h"
// #include <cmath>

// // Helper function to create a sphere mesh
// Mesh Mesh::createSphere(float radius, unsigned int slices, unsigned int stacks) {
//     std::vector<Vector3> vertices;
//     std::vector<Vector3> normals;
//     std::vector<unsigned int> indices;

//     for (unsigned int i = 0; i <= stacks; ++i) {
//         float stackAngle = glm::pi<float>() * float(i) / float(stacks); // From 0 to PI
//         for (unsigned int j = 0; j <= slices; ++j) {
//             float sliceAngle = 2.0f * glm::pi<float>() * float(j) / float(slices); // From 0 to 2PI

//             float x = radius * sin(stackAngle) * cos(sliceAngle);
//             float y = radius * cos(stackAngle);
//             float z = radius * sin(stackAngle) * sin(sliceAngle);

//             vertices.push_back(Vector3(x, y, z));
//             normals.push_back(Vector3(x, y, z).normalized()); // Normal is the same as the vertex position
//         }
//     }

//     // Create indices for the triangles
//     for (unsigned int i = 0; i < stacks; ++i) {
//         for (unsigned int j = 0; j < slices; ++j) {
//             unsigned int first = (i * (slices + 1)) + j;
//             unsigned int second = first + slices + 1;

//             indices.push_back(first);
//             indices.push_back(second);
//             indices.push_back(first + 1);

//             indices.push_back(second);
//             indices.push_back(second + 1);
//             indices.push_back(first + 1);
//         }
//     }

//     return Mesh(vertices, normals, indices);
// }

// // Helper function to create a cube mesh
// Mesh Mesh::createCube(float size) {
//     std::vector<Vector3> vertices = {
//         Vector3(-size, -size, -size), Vector3(size, -size, -size), Vector3(size, size, -size), Vector3(-size, size, -size),  // Front face
//         Vector3(-size, -size, size), Vector3(size, -size, size), Vector3(size, size, size), Vector3(-size, size, size),   // Back face
//     };

//     std::vector<Vector3> normals = {
//         Vector3(0, 0, -1), Vector3(0, 0, 1), Vector3(-1, 0, 0), Vector3(1, 0, 0),  // Normals for each face
//         Vector3(0, -1, 0), Vector3(0, 1, 0)
//     };

//     std::vector<unsigned int> indices = {
//         // Front face
//         0, 1, 2, 2, 3, 0,
//         // Back face
//         4, 5, 6, 6, 7, 4,
//         // Left face
//         0, 3, 7, 7, 4, 0,
//         // Right face
//         1, 2, 6, 6, 5, 1,
//         // Bottom face
//         0, 1, 5, 5, 4, 0,
//         // Top face
//         3, 2, 6, 6, 7, 3
//     };

//     return Mesh(vertices, normals, indices);
// }

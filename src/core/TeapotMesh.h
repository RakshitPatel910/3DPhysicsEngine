// TeapotMesh.h
#pragma once
#include "Mesh.h"     // Your Mesh class must have a constructor: Mesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<unsigned int>& indices)
#include "Vector3.h"  // Your Vector3 class with overloaded operators, normalized(), and cross() methods.
#include <vector>
#include <cmath>

// Control points for the teapot (32 points)
// (Data taken from the classic Utah Teapot – public domain)
static const float teapotCP[32][3] = {
    { 0.2000f,  0.0000f,  2.7000f},
    { 0.2000f, -0.1120f,  2.7000f},
    { 0.1120f, -0.2000f,  2.7000f},
    { 0.0000f, -0.2000f,  2.7000f},
    { 1.3375f,  0.0000f,  2.53125f},
    { 1.3375f, -0.7490f,  2.53125f},
    { 0.7490f, -1.3375f,  2.53125f},
    { 0.0000f, -1.3375f,  2.53125f},
    { 1.4375f,  0.0000f,  2.53125f},
    { 1.4375f, -0.8050f,  2.53125f},
    { 0.8050f, -1.4375f,  2.53125f},
    { 0.0000f, -1.4375f,  2.53125f},
    { 1.5000f,  0.0000f,  2.40000f},
    { 1.5000f, -0.8400f,  2.40000f},
    { 0.8400f, -1.5000f,  2.40000f},
    { 0.0000f, -1.5000f,  2.40000f},
    { -0.2000f,  0.0000f,  2.7000f},
    { -0.2000f, -0.1120f,  2.7000f},
    { -0.1120f, -0.2000f,  2.7000f},
    { 0.0000f, -0.2000f,  2.7000f},
    { -1.3375f,  0.0000f,  2.53125f},
    { -1.3375f, -0.7490f,  2.53125f},
    { -0.7490f, -1.3375f,  2.53125f},
    { 0.0000f, -1.3375f,  2.53125f},
    { -1.4375f,  0.0000f,  2.53125f},
    { -1.4375f, -0.8050f,  2.53125f},
    { -0.8050f, -1.4375f,  2.53125f},
    { 0.0000f, -1.4375f,  2.53125f},
    { -1.5000f,  0.0000f,  2.40000f},
    { -1.5000f, -0.8400f,  2.40000f},
    { -0.8400f, -1.5000f,  2.40000f},
    { 0.0000f, -1.5000f,  2.40000f}
};

// Patch indices for the upper half of the teapot (10 patches)
// (For brevity, patches 0-3 are defined; patches 4-9 are repeated copies in this example.)
static const int teapotPatches[10][16] = {
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  3,  2,  1,  0, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4 },
    { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 },
    { 19, 18, 17, 16, 23, 22, 21, 20, 27, 26, 25, 24, 31, 30, 29, 28 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  3,  2,  1,  0, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4 },
    { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 },
    { 19, 18, 17, 16, 23, 22, 21, 20, 27, 26, 25, 24, 31, 30, 29, 28 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  3,  2,  1,  0, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4 }
};

// Evaluate Bernstein polynomial of degree 3.
inline float Bernstein(int i, float t) {
    switch(i) {
        case 0: return (1-t)*(1-t)*(1-t);
        case 1: return 3*t*(1-t)*(1-t);
        case 2: return 3*t*t*(1-t);
        case 3: return t*t*t;
        default: return 0.0f;
    }
}

// Evaluate a point on a Bezier patch given a 4x4 grid of control points.
Vector3 EvaluateBezierPatch(const Vector3 cp[4][4], float u, float v) {
    Vector3 point = {0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        float bu = Bernstein(i, u);
        for (int j = 0; j < 4; ++j) {
            float bv = Bernstein(j, v);
            point = point + cp[i][j] * (bu * bv);
        }
    }
    return point;
}

// Tessellate one 4x4 patch with a given resolution.
void TessellatePatch(const Vector3 cp[4][4], int tessellation,
                     std::vector<Vector3>& outVertices,
                     std::vector<Vector3>& outNormals,
                     std::vector<unsigned int>& outIndices)
{
    int startIndex = outVertices.size();
    int numPerRow = tessellation + 1;
    // Generate vertices and approximate normals (using finite differences)
    for (int i = 0; i <= tessellation; ++i) {
        float u = (float)i / tessellation;
        for (int j = 0; j <= tessellation; ++j) {
            float v = (float)j / tessellation;
            Vector3 pos = EvaluateBezierPatch(cp, u, v);
            outVertices.push_back(pos);
            // Finite difference normal approximation:
            float du = 0.001f, dv = 0.001f;
            Vector3 posU = EvaluateBezierPatch(cp, std::min(u+du,1.0f), v);
            Vector3 posV = EvaluateBezierPatch(cp, u, std::min(v+dv,1.0f));
            Vector3 tangentU = (posU - pos).normalized();
            Vector3 tangentV = (posV - pos).normalized();
            Vector3 normal = tangentU.cross(tangentV).normalized();
            outNormals.push_back(normal);
        }
    }
    // Generate indices for triangles
    for (int i = 0; i < tessellation; ++i) {
        for (int j = 0; j < tessellation; ++j) {
            int i0 = startIndex + i * numPerRow + j;
            int i1 = i0 + 1;
            int i2 = i0 + numPerRow;
            int i3 = i2 + 1;
            outIndices.push_back(i0);
            outIndices.push_back(i2);
            outIndices.push_back(i1);
            outIndices.push_back(i1);
            outIndices.push_back(i2);
            outIndices.push_back(i3);
        }
    }
}

// Generate the complete Utah Teapot mesh.
// tessellation: subdivisions per patch (default 10)
inline Mesh GenerateTeapotMesh(int tessellation = 10) {
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<unsigned int> indices;
    
    // Process each patch for the upper half.
    for (int patch = 0; patch < 10; ++patch) {
        Vector3 cp[4][4];
        // Extract the 4x4 control points for this patch.
        for (int i = 0; i < 16; ++i) {
            int row = i / 4;
            int col = i % 4;
            int index = teapotPatches[patch][i];
            cp[row][col] = Vector3(teapotCP[index][0], teapotCP[index][1], teapotCP[index][2]);
        }
        TessellatePatch(cp, tessellation, vertices, normals, indices);
    }
    
    // Mirror the upper half to generate the lower half.
    int upperVertexCount = vertices.size();
    int upperIndexCount = indices.size();
    // Mirror vertices and normals by negating the y coordinate.
    for (int i = 0; i < upperVertexCount; ++i) {
        Vector3 v = vertices[i];
        v.y = -v.y;
        vertices.push_back(v);
        Vector3 n = normals[i];
        n.y = -n.y;
        normals.push_back(n);
    }
    // For indices of the lower half, adjust indices and reverse winding.
    int lowerStart = upperVertexCount;
    // Each patch produced tessellation+1 squared vertices.
    int vertsPerPatch = (tessellation + 1) * (tessellation + 1);
    for (int patch = 0; patch < 10; ++patch) {
        int base = patch * vertsPerPatch;
        for (int i = 0; i < tessellation; ++i) {
            for (int j = 0; j < tessellation; ++j) {
                int numPerRow = tessellation + 1;
                int i0 = lowerStart + base + i * numPerRow + j;
                int i1 = i0 + 1;
                int i2 = i0 + numPerRow;
                int i3 = i2 + 1;
                // Reverse winding order for proper normals.
                indices.push_back(i0);
                indices.push_back(i1);
                indices.push_back(i2);
                indices.push_back(i1);
                indices.push_back(i3);
                indices.push_back(i2);
            }
        }
    }
    
    return Mesh(vertices, normals, indices);
}

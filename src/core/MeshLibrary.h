#pragma once

#include "Vector3.h"
#include "Mesh.h"

#include <vector>
#include <cmath>


class MeshLibrary
{
public:
    static const Mesh& getCubeMesh() {
        static Mesh cube = []() -> Mesh {
            std::vector<Vector3> vertices = {
                {-0.5f, -0.5f,  0.5f},
                { 0.5f, -0.5f,  0.5f},
                { 0.5f,  0.5f,  0.5f},
                {-0.5f,  0.5f,  0.5f},
                {-0.5f, -0.5f, -0.5f},
                { 0.5f, -0.5f, -0.5f},
                { 0.5f,  0.5f, -0.5f},
                {-0.5f,  0.5f, -0.5f}
            };

            std::vector<Vector3> normals( vertices.size(), {0.0f, 0.0f, 1.0f} );

            std::vector<unsigned int> indices = {
                0, 1, 2, 2, 3, 0,  // front
                4, 5, 6, 6, 7, 4,  // back
                4, 0, 3, 3, 7, 4,  // left
                1, 5, 6, 6, 2, 1,  // right
                3, 2, 6, 6, 7, 3,  // top
                4, 5, 1, 1, 0, 4   // bottom
            };

            return Mesh(vertices, normals, indices);
        }();

        return cube;
    }

    static const Mesh& getSphereMesh( int sectors = 36, int stacks = 18 ) {
        static Mesh sphere = [sectors, stacks]() -> Mesh {
            std::vector<Vector3> vertices;
            std::vector<Vector3> normals;
            std::vector<unsigned int> indices;

            float x, y, z, xy;
            float secStep = 2 * M_PI / sectors;
            float staStep = M_PI / stacks;
            float secAngle, staAngle;

            for(int i = 0; i <= stacks; i++) {
                staAngle = M_PI / 2 - i * staStep; // from pi/2 to -pi/2

                xy = cosf(staAngle);
                z = sinf(staAngle);

                for(int j = 0; j <= sectors; j++) {
                    secAngle = j * secStep;

                    x = xy * cosf(secAngle);
                    y = xy * sinf(secAngle);

                    vertices.push_back({x, y, z});
                    normals.push_back({x, y, z}); // normals same as vertices because unit sphere
                }
            }

            int v1, v2;
            for(int i = 0; i <= stacks; i++) {
                v1 = i * (sectors + 1);
                v2 = v1 + sectors + 1;

                for(int j = 0; j <= sectors; j++, v1++, v2++) {
                    if(i != 0) {
                        indices.push_back(v1);
                        indices.push_back(v2);
                        indices.push_back(v1 + 1);
                    }

                    if(i != stacks - 1) {
                        indices.push_back(v1 + 1);
                        indices.push_back(v2);
                        indices.push_back(v2 + 1);
                    }
                }
            }

            return Mesh(vertices, normals, indices);
        }();

        return sphere;
    }

    static const Mesh& GetCylinderMesh(int sectorCount = 36) {
        static Mesh cylinder = [sectorCount]() -> Mesh {
            std::vector<Vector3> vertices;
            std::vector<Vector3> normals;
            std::vector<unsigned int> indices;

            float radius = 0.5f;
            float height = 1.0f;
            float sectorStep = 2 * M_PI / sectorCount;
            float sectorAngle;

            // Side vertices
            for (int i = 0; i <= 1; ++i) {
                float h = -height / 2 + i * height; // bottom and top
                for (int j = 0; j <= sectorCount; ++j) {
                    sectorAngle = j * sectorStep;
                    float x = radius * cosf(sectorAngle);
                    float y = radius * sinf(sectorAngle);
                    vertices.push_back({x, y, h});
                    normals.push_back({x, y, 0});
                }
            }

            // Indices for side faces
            int k1, k2;
            for (int i = 0; i < 1; ++i) {
                k1 = i * (sectorCount + 1);
                k2 = k1 + sectorCount + 1;
                for (int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
                    indices.push_back(k1);
                    indices.push_back(k2);
                    indices.push_back(k1 + 1);

                    indices.push_back(k1 + 1);
                    indices.push_back(k2);
                    indices.push_back(k2 + 1);
                }
            }

            // Top circle (fan)
            int topCenterIndex = vertices.size();
            vertices.push_back({0, 0, height / 2});
            normals.push_back({0, 0, 1});
            for (int j = 0; j < sectorCount; ++j) {
                sectorAngle = j * sectorStep;
                float x = radius * cosf(sectorAngle);
                float y = radius * sinf(sectorAngle);
                vertices.push_back({x, y, height / 2});
                normals.push_back({0, 0, 1});
            }
            for (int j = 0; j < sectorCount; ++j) {
                int current = topCenterIndex + 1 + j;
                int next = topCenterIndex + 1 + ((j + 1) % sectorCount);
                indices.push_back(topCenterIndex);
                indices.push_back(current);
                indices.push_back(next);
            }

            // Bottom circle (fan)
            int bottomCenterIndex = vertices.size();
            vertices.push_back({0, 0, -height / 2});
            normals.push_back({0, 0, -1});
            for (int j = 0; j < sectorCount; ++j) {
                sectorAngle = j * sectorStep;
                float x = radius * cosf(sectorAngle);
                float y = radius * sinf(sectorAngle);
                vertices.push_back({x, y, -height / 2});
                normals.push_back({0, 0, -1});
            }
            for (int j = 0; j < sectorCount; ++j) {
                int current = bottomCenterIndex + 1 + j;
                int next = bottomCenterIndex + 1 + ((j + 1) % sectorCount);
                indices.push_back(bottomCenterIndex);
                indices.push_back(next);
                indices.push_back(current);
            }

            return Mesh(vertices, normals, indices);
        }();
        return cylinder;
    }
};


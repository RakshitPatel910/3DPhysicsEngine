#pragma once

#include "Vector3.h"

#include <fstream>   // For file input/output (ifstream)
#include <iostream>  // For console output (std::cout, std::cerr)
#include <map>       // For std::map (vertex index deduplication)
#include <sstream>   // For std::istringstream (parsing lines)
#include <string>    // For std::string
#include <tuple>     // For std::tuple (key for the map, though not strictly needed in the final version)
#include <vector>    // For std::vector
#include <cmath>     // For sqrt() in Vector3::normalize()

class MeshLoader
{
    public:
    static Mesh* loadFromObj(const std::string& path) {
        std::vector<Vector3> temp_vertices;
        std::vector<Vector3> temp_normals;
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;
        std::vector<unsigned int> indices;

        std::map<std::tuple<int, int>, unsigned int> vertexIndices;

        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open OBJ file: " << path << std::endl;
            return nullptr;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;

            if (type == "v") {
                float x, y, z;
                iss >> x >> y >> z;
                temp_vertices.emplace_back(x, y, z);
            }
            else if (type == "vn") {
                float x, y, z;
                iss >> x >> y >> z;
                temp_normals.emplace_back(x, y, z);
            }
            else if (type == "f") {
                std::vector<std::string> faceData;
                std::string vertexData;
                while (iss >> vertexData) {
                    faceData.push_back(vertexData);
                }

                // Triangulate the face (converts quads/polygons to triangles)
                for (size_t i = 1; i < faceData.size() - 1; ++i) {
                    processFaceVertex(faceData[0], temp_vertices, temp_normals, vertices, normals, indices, vertexIndices);
                    processFaceVertex(faceData[i], temp_vertices, temp_normals, vertices, normals, indices, vertexIndices);
                    processFaceVertex(faceData[i+1], temp_vertices, temp_normals, vertices, normals, indices, vertexIndices);
                }
            }
        }

        return new Mesh(vertices, normals, indices);
    }

private:
    static void processFaceVertex(const std::string& vertexData,
                                  const std::vector<Vector3>& temp_vertices,
                                  const std::vector<Vector3>& temp_normals,
                                  std::vector<Vector3>& vertices,
                                  std::vector<Vector3>& normals,
                                  std::vector<unsigned int>& indices,
                                  std::map<std::tuple<int, int>, unsigned int>& vertexIndices) {
        std::istringstream viss(vertexData);
        std::string vPart, vtPart, vnPart;
        int vIdx = -1, vnIdx = -1;

        std::getline(viss, vPart, '/');
        std::getline(viss, vtPart, '/');
        std::getline(viss, vnPart, '/');

        vIdx = std::stoi(vPart) - 1;
        if (!vnPart.empty()) vnIdx = std::stoi(vnPart) - 1;

        auto key = std::make_tuple(vIdx, vnIdx);
        if (vertexIndices.find(key) == vertexIndices.end()) {
            vertexIndices[key] = static_cast<unsigned int>(vertices.size());
            vertices.push_back(temp_vertices[vIdx]);
            if (vnIdx >= 0 && vnIdx < temp_normals.size()) {
                normals.push_back(temp_normals[vnIdx]);
            } else {
                normals.emplace_back(0, 0, 0); // Default normal if missing
            }
        }
        indices.push_back(vertexIndices[key]);
    }
};


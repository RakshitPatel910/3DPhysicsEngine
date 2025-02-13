#pragma once

#include <iostream>
#include <vector>
// #include <GL/glew.h>
#include <glad/glad.h>
#include <glm/glm.hpp>


#include "Vector3.h"

// ! for better performance, change this to single buffer, instead of three different buffers
class Mesh
{
// private:
public:
    // struct Face
    // {
    //     unsigned int v1, v2, v3;
    // };

    std::vector<Vector3> vertices;
    // std::vector<Face> faces;
    std::vector<Vector3> normals;
    std::vector<unsigned int> indices;
    std::vector<unsigned int> lineIndices;

    GLuint VAO, VBO, EBO, normalBufffer, lineEBO;

    Mesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<unsigned int>& indices)
        : vertices(vertices), normals(normals), indices(indices) 
    {
        for(size_t i = 0; i < indices.size(); i += 3) {
            unsigned int v0 = indices[i];
            unsigned int v1 = indices[i + 1];
            unsigned int v2 = indices[i + 2];

            lineIndices.push_back(v0);
            lineIndices.push_back(v1);
            lineIndices.push_back(v1);
            lineIndices.push_back(v2);
            lineIndices.push_back(v2);
            lineIndices.push_back(v0);
        }

        setBuffers();
    }

    ~Mesh() {
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteBuffers(1, &normalBufffer);
        glDeleteVertexArrays(1, &VAO);
    }

    GLuint getVAO() const {
        return VAO;
    }

    void setBuffers() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        glGenBuffers(1, &normalBufffer);
        glGenBuffers(1, &lineEBO);

        glBindVertexArray(VAO);

        std::vector<glm::vec3> glmVer;
        glmVer.reserve(vertices.size());
        for(const Vector3& vec : vertices) {
            glmVer.push_back(glm::vec3(vec.x, vec.y, vec.z));
        }

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, glmVer.size() * sizeof(glm::vec3), glmVer.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
        glEnableVertexAttribArray(0);

        std::vector<glm::vec3> glmNorm;
        glmVer.reserve(normals.size());
        for(const Vector3& vec : normals) {
            glmNorm.push_back(glm::vec3(vec.x, vec.y, vec.z));
        }

        glBindBuffer(GL_ARRAY_BUFFER, normalBufffer);
        glBufferData(GL_ARRAY_BUFFER, glmNorm.size() * sizeof(glm::vec3), glmNorm.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lineEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, lineIndices.size() * sizeof(unsigned int), lineIndices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, EBO);

        glBindVertexArray(0);
    }

    // void draw(GLuint shaderProgram) {
    void draw() const {
        glBindVertexArray(VAO);

        // std::cout << VAO << '\n';
        // glUseProgram(shaderProgram);

        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        glBindVertexArray(0);
    }

    void drawWireframe() const {
        glBindVertexArray(VAO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lineEBO);
        
        glDrawElements(GL_LINES, lineIndices.size(), GL_UNSIGNED_INT, 0);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBindVertexArray(0);
    }

    void drawVertices() const {
        glBindVertexArray(VAO);

        glDrawArrays(GL_POINTS, 0, vertices.size());

        glBindVertexArray(0);
    }

    void drawInstanced(unsigned int instCount) const {
        glBindVertexArray(VAO);
        glDrawElementsInstanced(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0, instCount);

        glBindVertexArray(0);
    }
};


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

    GLuint VAO, VBO, EBO, normalBufffer;

    Mesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<unsigned int>& indices)
        : vertices(vertices), normals(normals), indices(indices) 
    {
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
        glmVer.reserve(vertices.size());
        for(const Vector3& vec : normals) {
            glmNorm.push_back(glm::vec3(vec.x, vec.y, vec.z));
        }

        glBindBuffer(GL_ARRAY_BUFFER, normalBufffer);
        glBufferData(GL_ARRAY_BUFFER, glmNorm.size() * sizeof(glm::vec3), glmNorm.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (GLvoid*)0);
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
    }

    // void draw(GLuint shaderProgram) {
    void draw() const {
        glBindVertexArray(VAO);

        // std::cout << VAO << '\n';
        // glUseProgram(shaderProgram);

        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        glBindVertexArray(0);
    }

    void drawInstanced(unsigned int instCount) const {
        glBindVertexArray(VAO);
        glDrawElementsInstanced(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0, instCount);

        glBindVertexArray(0);
    }
};


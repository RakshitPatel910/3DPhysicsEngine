#pragma once 

#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>

#include "Camera.h"
#include "Shader.h"
#include "../core/Matrix4.h"
#include "../core/Mesh.h"

class Renderer
{
public:
    
    struct RenderBatch {
        const Mesh* mesh;
        std::vector<glm::mat4> transformMats;
    }

    void initialize(int w, int h) {
        screenW = w;
        screenH = h;

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
    }

    void submitMesh(const Mesh& mesh, const glm::mat4& transform) {
        batchList[&mesh].mesh = &mesh;
        batchList[&mesh].tansformMats.push_back(transform);
    }

    void beginFrame() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        batches.clear();
    }

    void endFrame(const Camera& camera) {
        defaultShader.bind();
        
        defaultShader.setUnifMat4("view", camera.getViewMatrix());
        // defaultShader.setUnifMat4("projection", camera.) //TODO: implement getProjectionMatrix() in camera class

        for( auto& [meshPtr, batch] : batchList ) {
            setupInstancedAttributtes(*meshPtr, batch.transformMats);
            meshPtr->drawInstanced(batchList.transformMats.size());
        }
    }

private:
    std::unordered_map<Mesh*, RenderBatch> batchList;
    Shader defaultShader("shaders/default.vert", "shaders/default.frag"); //! dont know if this is correct
    int screenW, screenH;


    void setupInstancedAttributtes(const Mesh& mesh, const vector<glm::mat4>& transforms) {
        GLuint currInstanceVBO

        glGenBuffers(1, &currInstanceVBO);
        glBindBuffer(GL_ARRAY_BUFFER, currInstanceVBO);
        glBufferData(GL_ARRAY_BUFFER, trasforms.size() * sizeof(glm::mat4), transforms.data(), GL_STATIC_DRAW);
        glBindVertexArray(currInstanceVBO);

        for(int i = 0; i < 4; i++){
            glEnableVertexAttribArray(2 + i);
            glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(i * sizeof(glm::vec4)));
            glVertexAttribDivisor(2 + i, 1);
        }

        glBindBuffer(0);
        glDeleteBuffers(1, &currInstanceVBO);
    }
};
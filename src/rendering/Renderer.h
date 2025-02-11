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
    };

    struct RenderCommand {
        const Mesh* mesh;
        glm::mat4 transform;
    };

    Renderer() : defaultShader("../src/rendering/resources/default.vert", "../src/rendering/resources/default.frag") {};

    void initialize(int w, int h) {
        screenW = w;
        screenH = h;

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        // glEnable(GL_CULL_FACE);
    }

    // void submitMesh(const Mesh& mesh, const glm::mat4& transform) {
    //     batchList[&mesh].mesh = &mesh;
    //     batchList[&mesh].transformMats.push_back(transform);
    // }
    // void submitMesh(Mesh* mesh, const glm::mat4& transform) {
    //     batchList[mesh].mesh = mesh;
    //     batchList[mesh].transformMats.push_back(transform);
    // }
    void submitMesh(const Mesh& mesh, const glm::mat4& transform) {
        commands.push_back({&mesh, transform});
    }

    void beginFrame() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        batchList.clear();
    }

    void endFrame(const Camera& camera) {
        defaultShader.bind();
        
        defaultShader.setUnifMat4("view", camera.getViewMatrix());
        // defaultShader.setUnifMat4("projection", camera.getProjectionMatrix()); // TODO: implement getProjectionMatrix() in camera class
        defaultShader.setUnifMat4("projection", 
            camera.getProjectionMatrix(static_cast<float>(screenW)/screenH));

        // for (auto& [meshPtr, batch] : batchList) {
        //     setupInstancedAttributes(*meshPtr, batch.transformMats);
        //     meshPtr->drawInstanced(batch.transformMats.size());
        // }

        for(const auto& cmd : commands) {
            defaultShader.setUnifMat4("model", cmd.transform);
            cmd.mesh->draw();
        }

        commands.clear();
    }

private:
    std::unordered_map<Mesh*, RenderBatch> batchList;
    std::vector<RenderCommand> commands;
    // Shader defaultShader("rendering/resources/default.vert", "rendering/resources/default.frag"); //! Make sure the paths to the shaders are correct
    Shader defaultShader;
    int screenW, screenH;

    void setupInstancedAttributes(const Mesh& mesh, const std::vector<glm::mat4>& transforms) {
        GLuint currInstanceVBO;
        glGenBuffers(1, &currInstanceVBO);
        glBindBuffer(GL_ARRAY_BUFFER, currInstanceVBO);
        glBufferData(GL_ARRAY_BUFFER, transforms.size() * sizeof(glm::mat4), transforms.data(), GL_STATIC_DRAW);
        glBindVertexArray(currInstanceVBO);

        for (int i = 0; i < 4; i++) {
            glEnableVertexAttribArray(2 + i);
            glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(i * sizeof(glm::vec4)));
            glVertexAttribDivisor(2 + i, 1);
        }

        // glBindBuffer(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        
        glDeleteBuffers(1, &currInstanceVBO);
    }
};
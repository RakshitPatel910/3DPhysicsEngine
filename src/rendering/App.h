#pragma once

#include "Window.h"
#include "Camera.h"
#include "Renderer.h"
#include "../core/Vector3.h"
#include "../core/Matrix4.h"
#include "../core/Mesh.h"

class App {
public:
    App() : window(1280, 720, "Physics Engine"),
            renderer(),
            camera(Camera()) 
    {
        // camera.pos = Vector3(0, 0, 5);
        camera.pos.printV();
        setupCallbacks();
        renderer.initialize(windowWidth, windowHeight);
    }

    void run() {
        double lastTime = glfwGetTime();
        
        while(!window.shouldClose()) {
            // Calculate delta time
            double currentTime = glfwGetTime();
            float deltaTime = static_cast<float>(currentTime - lastTime);
            lastTime = currentTime;

            // Update
            processInput(deltaTime);

            // Render
            renderer.beginFrame();
            renderScene();
            renderer.endFrame(camera);
            
            window.swapBuffers();
            window.pollEvents();
        }
    }

private:
    Window window;
    Renderer renderer;
    Camera camera;
    int windowWidth = 1280;
    int windowHeight = 720;
    double lastMouseX = 0;
    double lastMouseY = 0;
    bool firstMouse = true;

    void setupCallbacks() {
        window.setResizeCallback([this](int w, int h) {
            windowWidth = w;
            windowHeight = h;
            glViewport(0, 0, w, h);
        });

        window.setMouseCallback([this](double x, double y) {
            if(glfwGetMouseButton(window.getHandle(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                if(firstMouse) {
                    lastMouseX = x;
                    lastMouseY = y;
                    firstMouse = false;
                }
                
                double xoffset = x - lastMouseX;
                double yoffset = lastMouseY - y; // Reversed y
                lastMouseX = x;
                lastMouseY = y;
                
                camera.processLook(xoffset*0.01, yoffset*0.01);
            }
        });
    }

    void processInput(float deltaTime) {
        Vector3 movement(0, 0, 0);
        
        if(glfwGetKey(window.getHandle(), GLFW_KEY_W) == GLFW_PRESS)
            movement.z += 1;
        if(glfwGetKey(window.getHandle(), GLFW_KEY_S) == GLFW_PRESS)
            movement.z -= 1;
        if(glfwGetKey(window.getHandle(), GLFW_KEY_A) == GLFW_PRESS)
            movement.x -= 1;
        if(glfwGetKey(window.getHandle(), GLFW_KEY_D) == GLFW_PRESS)
            movement.x += 1;
        if(glfwGetKey(window.getHandle(), GLFW_KEY_Q) == GLFW_PRESS)
            movement.y += 1;
        if(glfwGetKey(window.getHandle(), GLFW_KEY_E) == GLFW_PRESS)
            movement.y -= 1;

        camera.processMovement(movement, deltaTime);
    }

    void renderScene() {
        static Mesh cube = createCubeMesh();
        static glm::mat4 cubeTransform = (Matrix4() * Quaternion(3.14159265359f / 2.5f, 0, 1, 0).toMatrix4()).toGlm();
        
        // std::cout << glm::to_string(camera.getViewMatrix()) << std::endl;
        // Matrix4 m = Matrix4();
        // m = m.fromGlm(camera.getViewMatrix());
        // m.printMat();
        // glm::mat4 m = camera.getViewMatrix();
        // for (int i = 0; i < 4; ++i) {
        //     for (int j = 0; j < 4; ++j) {
        //         std::cout << m[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        // std::cout << "cam pos "; camera.pos.printV();

        // cubeTransform.rotation.y += 1.0f;
        renderer.submitMesh(cube, cubeTransform);
    }

    // Mesh createCubeMesh() {
    //     // Implement cube vertex data
    //     std::vector<Vector3> vertices = {...};
    //     std::vector<Vector3> normals = {...};
    //     std::vector<unsigned int> indices = {...};
        
    //     return Mesh(vertices, normals, indices);
    // }
    Mesh createCubeMesh() {
        std::vector<Vector3> vertices = {
            // Front face
            {-0.5f, -0.5f,  0.5f}, {0.5f, -0.5f,  0.5f}, {0.5f,  0.5f,  0.5f},
            {-0.5f,  0.5f,  0.5f},
            // Back face
            {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f,  0.5f, -0.5f},
            {-0.5f,  0.5f, -0.5f}
        };
    
        std::vector<Vector3> normals(vertices.size(), {0, 0, 1}); // Simplified normals
    
        std::vector<unsigned int> indices = {
            // Front
            0, 1, 2, 2, 3, 0,
            // Back
            4, 5, 6, 6, 7, 4,
            // Left
            4, 0, 3, 3, 7, 4,
            // Right
            1, 5, 6, 6, 2, 1,
            // Top
            3, 2, 6, 6, 7, 3,
            // Bottom
            4, 5, 1, 1, 0, 4
        };
    
        return Mesh(vertices, normals, indices);
    }
};
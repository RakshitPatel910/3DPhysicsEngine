#pragma once

#include "Window.h"
#include "Camera.h"
#include "Renderer.h"
#include "../core/Vector3.h"
#include "../core/Matrix4.h"
#include "../core/Mesh.h"
#include "../core/MeshLibrary.h"
#include "../core/MeshLoader.h"

// #include "../core/Vector3.h"
// #include "../core/Matrix4.h"
#include "../core/Quaternion.h"
#include "../core/RigidBody.h"
#include "../core/AABB.h"
#include "../core/Collider.h"
#include "../core/shape/Shape.h"
#include "../core/PhysicsWorld.h"

class App {
public:
    App() : window(1280, 720, "Physics Engine"),
            renderer(),
            camera(Camera()) 
    {
        camera.pos = Vector3(0, 0, 50);
        camera.pos.printV();
        setupCallbacks();
        renderer.initialize(windowWidth, windowHeight);
    }

    void run() {
        double lastTime = glfwGetTime();
        
        // Cube cube1(Vector3(2, 3, 1));
        // Cube cube2(Vector3(2, 3, 1));

        // Cube cube1(Vector3(1.0f, 1.0f, 1.0f));
        // Cube cube2(Vector3(1.4f, 0.1f, 1.0f));

        Sphere cube1(2);
        Sphere cube2(2);

        RigidBody rb1(5.0f, &cube1, Vector3(-20,2,0.0f), Vector3(0.5f,0, 0), "cube 1");
        RigidBody rb2(5.0f, &cube2, Vector3(20,0,0.0f), Vector3(-0.5f,0, 0), "cube 2");

        // RigidBody rb1(5.0f, &cube1, Vector3(0.0f, 0.0f, 0.0f), Vector3(0,0, 0), "cube 1");
        // RigidBody rb2(5.0f, &cube2, Vector3(2.0f, 0.4f, 0.0f), Vector3(0,0, 0), "cube 2");

        std::shared_ptr<RigidBody> rb1ptr= std::make_shared<RigidBody>(rb1);
        std::shared_ptr<RigidBody> rb2ptr= std::make_shared<RigidBody>(rb2);

        Collider cl1(5.0f, rb1.inertiaTensor, Vector3(), rb1ptr, &cube1, Collider::ColliderType::DYNAMIC);
        Collider cl2(5.0f, rb2.inertiaTensor, Vector3(), rb2ptr, &cube2, Collider::ColliderType::DYNAMIC);

        AABB aabb1 = AABB::fromHalfCentralExtents(rb1.getPosition(), Vector3(2,3,1));
        AABB aabb2 = AABB::fromHalfCentralExtents(rb2.getPosition(), Vector3(2,3,1));

        PhysicsWorld physicsWorld(&renderer);

        std::shared_ptr<Collider> cl1ptr = std::make_shared<Collider>(cl1);
        std::shared_ptr<Collider> cl2ptr = std::make_shared<Collider>(cl2);

        physicsWorld.addRigidBody(rb1ptr, cl1ptr);
        physicsWorld.addRigidBody(rb2ptr, cl2ptr);

        // PhysicsWorld.simulate();

        // physicsWorld.simulate();


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

            physicsWorld.simulate();

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

    // const Mesh* teapot = MeshLoader::loadFromObj("../src/rendering/resources/teapot.obj");
    // const Mesh* teapot = MeshLoader::loadFromObj("../src/rendering/resources/donut_icing.obj");

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
                
                camera.processLook(xoffset, yoffset);
            }

            if(glfwGetMouseButton(window.getHandle(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE) {
                firstMouse = true;
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
        // static Mesh cube = createCubeMesh();
        // const Mesh& cube = MeshLibrary::getCubeMesh();
        // const Mesh& sphere = MeshLibrary::getSphereMesh();
        // const Mesh& cylinder = MeshLibrary::GetCylinderMesh();

        // static glm::mat4 cubeTransform = (Matrix4() * Quaternion(3.14159265359f / 2.5f, 0, 1, 0).toMatrix4()).toGlm();
        // static glm::mat4 sphereTransform = (Matrix4::getTranslationMatrix(Vector3(2.5,-0.5,0))).toGlm();
        // static glm::mat4 cylinderTransform = (Matrix4::getTranslationMatrix(Vector3(0.75,2,0))).toGlm();
        // static glm::mat4 teapotTransform = Matrix4().toGlm();
        
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
        // renderer.submitMesh(cube, cubeTransform);
        // renderer.submitMesh(sphere, sphereTransform);
        // renderer.submitMesh(cylinder, cylinderTransform);
        // renderer.submitMesh(*teapot, teapotTransform);
    }

};
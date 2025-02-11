#pragma once

#include <iostream>
#include <glm/glm.hpp>

#include "../core/Vector3.h"
#include "../core/Quaternion.h"

class Camera
{
public:
    Vector3 pos = Vector3(0.0f, 0.0f, 3.0f);
    Quaternion orientation = Quaternion();

    float fov = 45.0f; // Field of View
    float moveSpeed = 5.0f;
    float sensitivity = 0.1f;

    // Camera();
    ~Camera() {};

    // get some Vector3 for camera
    Vector3 front() const {
        // Vector3 fr = orientation.rotateVec(Vector3(0.0f, 0.0f, -1.0f));
        // fr.printV();

        // return fr;
        return orientation.rotateVec(Vector3(0.0f, 0.0f, -1.0f));
        // return Vector3(0,0,-1);
    }

    Vector3 right() const {
        return orientation.rotateVec(Vector3(1.0f, 0.0f, 0.0f));
        // return Vector3(1,0,0);
    }

    Vector3 up() const {
        return orientation.rotateVec(Vector3(0.0f, 1.0f, 0.0f));
        // return Vector3(0,1,0);
    }

    glm::mat4 getViewMatrix() const {
        // std::cout << "p : "; pos.printV();
        // std::cout << "q : "; orientation.printQ();
        // std::cout << "f : "; front().printV();
        // std::cout << "u : "; up().printV();

        return glm::lookAt(
            glm::vec3(pos),
            glm::vec3(pos + front()),
            glm::vec3(up())
        );
    }

    glm::mat4 getProjectionMatrix(float aspectRatio) const {
        // // return glm::perspective(

        // // ); //TODO: complete this
        // return glm::mat4();
        return glm::perspective(
            glm::radians(fov),
            aspectRatio,
            0.1f,
            100.0f
        );
    }

    void processMovement(const Vector3& dir, float dt) {
        Vector3 velocity = dir * (this->moveSpeed * dt);
        
        pos += front() * velocity.z;
        pos += right() * velocity.x;
        pos += up() * velocity.y;
        // pos.printV();
    }

    void processLook(float x_offset, float y_offset) {
        x_offset *= sensitivity * 0.01;
        y_offset *= sensitivity * 0.01;

        Quaternion yawRotationQ(x_offset, 0.0f, 1.0f, 0.0f); // Global Y-axis
        Quaternion pitchRotationQ(y_offset, 1.0f, 0.0f, 0.0f); // Local Right-axis

        // std::cout << "orientation : "; orientation.printQ();
        // std::cout << "yaw : "; yawRotationQ.printQ();
        // std::cout << "pitch : "; pitchRotationQ.printQ();

        orientation = yawRotationQ * orientation * pitchRotationQ; // NewOrientation = ΔYawGlobal × CurrentOrientation × ΔPitchLocal
        orientation.normalize();
    }
    // void processLook(float x_offset, float y_offset) {
    //     x_offset *= sensitivity*0.01;
    //     y_offset *= sensitivity*0.01;
    
    //     // Print initial orientation
    //     std::cout << "Initial Orientation Quaternion: ";
    //     orientation.printQ();
    
    //     // Create yaw rotation quaternion (rotation around Y-axis)
    //     Quaternion yawRotationQ(x_offset, 0.0f, 1.0f, 0.0f);
    //     std::cout << "Yaw Quaternion: ";
    //     yawRotationQ.printQ();
    
    //     // Apply yaw rotation to orientation and normalize
    //     orientation = yawRotationQ * orientation;
    //     orientation.normalize();
    //     std::cout << "Orientation after yaw rotation: ";
    //     orientation.printQ();
    
    //     // Create pitch rotation quaternion (rotation around X-axis)
    //     Quaternion pitchRotationQ(y_offset, 1.0f, 0.0f, 0.0f);
    //     std::cout << "Pitch Quaternion: ";
    //     pitchRotationQ.printQ();
    
    //     // Apply pitch rotation to orientation and normalize
    //     orientation = orientation * pitchRotationQ;
    //     orientation.normalize();
    //     std::cout << "Orientation after pitch rotation: ";
    //     orientation.printQ();
    
    //     // Final orientation quaternion print
    //     std::cout << "Final Orientation: ";
    //     orientation.printQ();
    // }
    

};


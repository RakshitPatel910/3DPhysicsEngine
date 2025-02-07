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

    float fov = 60.0f; // Field of View
    float moveSpeed = 5.0f;
    float sensitivity = 0.1f;

    // Camera();
    ~Camera() {};

    // get some Vector3 for camera
    Vector3 front() const {
        return orientation.rotateVec(Vector3(0.0f, 0.0f, -1.0f));
    }

    Vector3 right() const {
        return orientation.rotateVec(Vector3(1.0f, 0.0f, 0.0f));
    }

    Vector3 up() const {
        return orientation.rotateVec(Vector3(0.0f, 1.0f, 0.0f));
    }

    glm::mat4 getViewMatrix() const {
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
        
        // pos += front() * velocity.z;
        // pos += right() * velocity.x;
        // pos += up() * velocity.y;
        pos.printV();
    }

    void processLook(float x_offset, float y_offset) {
        // x_offset *= sensitivity;
        // y_offset *= sensitivity;

        // Quaternion yawRotationQ(x_offset, Vector3(0.0f, 1.0f, 0.0f)); // Global Y-axis
        // Quaternion pitchRotationQ(y_offset, right()); // Local Right-axis

        // orientation = yawRotationQ * orientation * pitchRotationQ; // NewOrientation = ΔYawGlobal × CurrentOrientation × ΔPitchLocal
        // orientation.normalize();
    }

};


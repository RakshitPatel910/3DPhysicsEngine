#pragma once

#include <glm/glm.hpp>

#include "../core/Vector3.h"
#include "../core/Quaternion.h"

class Camera
{
public:
    Vector3 pos = Vector3(0.0f, 0.0f, 3.0f);
    Quaternion orientation;

    float fov = 60.0f // Field of View
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
        Vector3 f = front();
        Vector3 r = right();
        Vector3 u = up();
    
        glm::mat4 view(1.0f);
        view[0][0] = r.x; view[1][0] = r.y; view[2][0] = r.z;
        view[0][1] = u.x; view[1][1] = u.y; view[2][1] = u.z;
        view[0][2] = -f.x; view[1][2] = -f.y; view[2][2] = -f.z;
        view[3][0] = -r.dot(position);
        view[3][1] = -u.dot(position);
        view[3][2] = -f.dot(position);

        return view;
    }

    void processMovement(const Vector& dir, float dt) {
        Vector3 velocity = dir * (moveSpeed * dt);

        pos += front() * velocity.z;
        pos += right() * velocity.x;
        pos += up() * velocity.y;
    }

    void processLook(float x_offset, float y_offset) {
        x_offset *= sensitivity;
        y_offset *= sensitivity;

        Quaternion yawRotationQ(x_offset, Vector3(0.0f, 1.0f, 0.0f)); // Global Y-axis
        Quaternion pitchRotationQ(y_offset, right()); // Local Right-axis

        orientation = yawRotationQ * orientation * pitchRotationQ; // NewOrientation = ΔYawGlobal × CurrentOrientation × ΔPitchLocal
        orientation.normalize();
    }

};


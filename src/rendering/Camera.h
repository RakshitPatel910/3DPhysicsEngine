#pragma once

#include <glm/glm.hpp>

#include "../core/Vector3.h"

class Camera
{
public:
    Vector3 pos = Vector3(0.0f, 0.0f, 3.0f);
    float fov = 60.0f // Field of View

    Camera(/* args */);
    ~Camera();
};


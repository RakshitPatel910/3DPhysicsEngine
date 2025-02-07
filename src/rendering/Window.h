#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <functional>

class Window
{
public:
    using ResizeCallback = std::function<void(int, int)>;
    using MouseCallback = std::function<void(double, double)>;

    Window(int width, int height, const char* title) {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
        window = glfwCreateWindow(width, height, title, nullptr, nullptr);
        glfwMakeContextCurrent(window);
        gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    }

    ~Window() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    GLFWwindow* getHandle() { 
        return window; 
    }

    void setResizeCallback(ResizeCallback cb) {
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, [](GLFWwindow* win, int w, int h) {
            auto self = static_cast<Window*>(glfwGetWindowUserPointer(win));
            self->resizeCallback(w, h);
        });

        resizeCallback = cb;
    }

    void setMouseCallback(MouseCallback cb) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSetWindowUserPointer(window, this);
        glfwSetCursorPosCallback(window, [](GLFWwindow* win, double x, double y) {
            auto self = static_cast<Window*>(glfwGetWindowUserPointer(win));
            self->mouseCallback(x, y);
        });

        mouseCallback = cb;
    }

    bool shouldClose() const {
        return glfwWindowShouldClose(window);
    }

    void pollEvents() {
        glfwPollEvents();
    }

    void swapBuffers() {
        glfwSwapBuffers(window);
    }

private:
    GLFWwindow* window;
    ResizeCallback resizeCallback;
    MouseCallback mouseCallback;
};


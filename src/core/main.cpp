#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "Vector3.h"
#include "Matrix4.h"
#include "Quaternion.h"
#include "RigidBody.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Global variables for the camera and box body
RigidBody boxBody;  // Make boxBody global
GLuint shaderProgram;

glm::mat4 viewMatrix;  // Camera's view matrix
glm::mat4 projectionMatrix;  // Projection matrix

// Camera settings
glm::vec3 cameraPosition(0.0f, 0.0f, 1000.0f);  // Start camera at z=10 (move it back)
float cameraSpeed = 10.0f;  // Camera movement speed
float cameraRotationSpeed = 0.05f;  // Camera rotation speed

// Camera controls (rotation)
float cameraYaw = -90.0f;  // Yaw (rotation around Y axis)
float cameraPitch = 0.0f;  // Pitch (rotation around X axis)

// Initialize OpenGL
bool initGL(int argc, char** argv, int width, int height) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("Physics Simulation");

    // Initialize GLEW
    glewInit();

    // Set up OpenGL states
    glEnable(GL_DEPTH_TEST);

    // Projection matrix setup
    projectionMatrix = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 2000.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projectionMatrix));

    // Initial view matrix setup (camera position and look-at)
    glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);  // Looking at the origin
    glm::vec3 upDirection(0.0f, 1.0f, 0.0f);  // Y-axis as up
    viewMatrix = glm::lookAt(cameraPosition, cameraTarget, upDirection);  // Camera matrix

    return true;
}

// Update the view matrix based on camera position and orientation
void updateViewMatrix() {
    glm::vec3 direction;
    direction.x = cos(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch));
    direction.y = sin(glm::radians(cameraPitch));
    direction.z = sin(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch));

    glm::vec3 cameraTarget = cameraPosition + glm::normalize(direction);
    glm::vec3 upDirection(0.0f, 1.0f, 0.0f);  // Y-axis as up
    viewMatrix = glm::lookAt(cameraPosition, cameraTarget, upDirection);  // Camera matrix
}

// Handle camera movement using keyboard
void keyboard(unsigned char key, int x, int y) {
    if (key == 'w') {
        cameraPosition += glm::normalize(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw)))) * cameraSpeed;
    } else if (key == 's') {
        cameraPosition -= glm::normalize(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw)))) * cameraSpeed;
    } else if (key == 'a') {
        cameraPosition -= glm::normalize(glm::cross(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw))), glm::vec3(0.0f, 1.0f, 0.0f))) * cameraSpeed;
    } else if (key == 'd') {
        cameraPosition += glm::normalize(glm::cross(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw))), glm::vec3(0.0f, 1.0f, 0.0f))) * cameraSpeed;
    }
    // Moving up (Q) and down (E)
    else if (key == 'q') {
        cameraPosition.y += cameraSpeed;
    } else if (key == 'e') {
        cameraPosition.y -= cameraSpeed;
    }
    updateViewMatrix();  // Update view matrix after moving camera
    glutPostRedisplay();  // Request to update the display
}

// Handle camera rotation using arrow keys
void specialKeys(int key, int x, int y) {
    if (key == GLUT_KEY_UP) {
        cameraPitch += cameraRotationSpeed;
    } else if (key == GLUT_KEY_DOWN) {
        cameraPitch -= cameraRotationSpeed;
    } else if (key == GLUT_KEY_LEFT) {
        cameraYaw -= cameraRotationSpeed;
    } else if (key == GLUT_KEY_RIGHT) {
        cameraYaw += cameraRotationSpeed;
    }
    updateViewMatrix();  // Update view matrix after rotating the camera
    glutPostRedisplay();  // Request to update the display
}

// Render the box
void renderBox(const RigidBody& body) {
    // Get position and transformation matrix from the RigidBody
    glm::vec3 position = glm::vec3(body.getPosition().getX(), body.getPosition().getY(), body.getPosition().getZ());
    glm::mat4 transformationMatrix = body.getTransformMatrix().toGlm(); // Assuming you have a toGlm() method

    glPushMatrix();

    // Apply the transformation matrix (translation + rotation + scaling)
    glMultMatrixf(glm::value_ptr(transformationMatrix));

    // Draw the cube (static geometry with dynamic transformations)
    glBegin(GL_QUADS);

    // Front face (Red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f( 0.5f, -0.5f, 0.5f);
    glVertex3f( 0.5f,  0.5f, 0.5f);
    glVertex3f(-0.5f,  0.5f, 0.5f);

    // Back face (Green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);
    glVertex3f( 0.5f, -0.5f, -0.5f);

    // Top face (Blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-0.5f,  0.5f,  0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);

    // Bottom face (Yellow)
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex3f(-0.5f, -0.5f,  0.5f);
    glVertex3f(-0.5f, -0.5f, -0.5f);
    glVertex3f( 0.5f, -0.5f, -0.5f);
    glVertex3f( 0.5f, -0.5f,  0.5f);

    // Right face (Magenta)
    glColor3f(1.0f, 0.0f, 1.0f);
    glVertex3f( 0.5f, -0.5f,  0.5f);
    glVertex3f( 0.5f, -0.5f, -0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);

    // Left face (Cyan)
    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f(-0.5f, -0.5f,  0.5f);
    glVertex3f(-0.5f,  0.5f,  0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);
    glVertex3f(-0.5f, -0.5f, -0.5f);

    glEnd();

    glPopMatrix();
}

// Display callback
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set the camera matrix (view matrix)
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(viewMatrix));

    // Render the box
    renderBox(boxBody);

    // Swap buffers
    glutSwapBuffers();
}

// Update function to simulate physics
void update(int value) {
    // boxBody.applyForce(Vector3(0.0f, -50.0f, 0.0f), Vector3(0.0f, 0.3f, 0.3f));

    // Integrate physics
    boxBody.integratePhysics(0.016f);

    // Request next frame update
    glutTimerFunc(16, update, 0);
    glutPostRedisplay();
}

// Main function
int main(int argc, char** argv) {
    if (!initGL(argc, argv, 800, 600)) {
        return -1;
    }

    // Create the box shape and rigid body
    BoxShape box = BoxShape(10.0f, 10.0f, 10.0f, 1.0f);  // 1x1x1 box
    boxBody = RigidBody(box, Vector3(0.0f, 0.0f, 0.0f));  // Start position at (0, 0, -10)

    // Apply some force to the box body for testing
    boxBody.applyForce(Vector3(100.0f, 100.0f, 0.0f), Vector3(0.0f, 0.3f, 0.3f));

    // Set GLUT callbacks
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutTimerFunc(16, update, 0);

    // Start GLUT main loop
    glutMainLoop();

    return 0;
}

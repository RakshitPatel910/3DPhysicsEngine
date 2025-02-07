// #include <glad/glad.h>
// // #include <GL/glew.h>
// // #include <GL/freeglut.h>
// #include <iostream>
// #include <vector>
// #include <cmath>
// #include "Vector3.h"
// #include "Matrix4.h"
// #include "Quaternion.h"
// #include "RigidBody.h"
// #include "AABBTree.h"
// #include "Collider.h"
// #include "AABB.h"
// #include "shape/Shape.h"
// #include <glm/glm.hpp>
// #include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtc/type_ptr.hpp>

// // Global variables for the camera and box body
// RigidBody boxBody;  // Make boxBody global
// std::vector<RigidBody> rigidBodies; // Vector to store rigid bodies
// std::vector<AABB> aabbList; // Vector to store rigid bodies
// AABB aabb1;
// AABB aabb2;
// AABB aabb3;
// AABB aabb4;
// AABB aabb5;

// GLuint shaderProgram;

// glm::mat4 viewMatrix;  // Camera's view matrix
// glm::mat4 projectionMatrix;  // Projection matrix

// // Camera settings
// glm::vec3 cameraPosition(0.0f, 0.0f, 1000.0f);  // Start camera at z=10 (move it back)
// float cameraSpeed = 10.0f;  // Camera movement speed
// float cameraRotationSpeed = 0.05f;  // Camera rotation speed

// // Camera controls (rotation)
// float cameraYaw = -90.0f;  // Yaw (rotation around Y axis)
// float cameraPitch = 0.0f;  // Pitch (rotation around X axis)

// AABBTree broadphase;

// // Initialize OpenGL
// bool initGL(int argc, char** argv, int width, int height) {
//     // Initialize GLUT
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(width, height);
//     glutCreateWindow("Physics Simulation");

//     // Initialize GLEW
//     glewInit();

//     // Set up OpenGL states
//     glEnable(GL_DEPTH_TEST);

//     // Projection matrix setup
//     projectionMatrix = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 2000.0f);
//     glMatrixMode(GL_PROJECTION);
//     glLoadMatrixf(glm::value_ptr(projectionMatrix));

//     // Initial view matrix setup (camera position and look-at)
//     glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);  // Looking at the origin
//     glm::vec3 upDirection(0.0f, 1.0f, 0.0f);  // Y-axis as up
//     viewMatrix = glm::lookAt(cameraPosition, cameraTarget, upDirection);  // Camera matrix

//     return true;
// }

// // Update the view matrix based on camera position and orientation
// void updateViewMatrix() {
//     glm::vec3 direction;
//     direction.x = cos(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch));
//     direction.y = sin(glm::radians(cameraPitch));
//     direction.z = sin(glm::radians(cameraYaw)) * cos(glm::radians(cameraPitch));

//     glm::vec3 cameraTarget = cameraPosition + glm::normalize(direction);
//     glm::vec3 upDirection(0.0f, 1.0f, 0.0f);  // Y-axis as up
//     viewMatrix = glm::lookAt(cameraPosition, cameraTarget, upDirection);  // Camera matrix
// }

// // Handle camera movement using keyboard
// void keyboard(unsigned char key, int x, int y) {
//     if (key == 'w') {
//         cameraPosition += glm::normalize(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw)))) * cameraSpeed;
//     } else if (key == 's') {
//         cameraPosition -= glm::normalize(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw)))) * cameraSpeed;
//     } else if (key == 'a') {
//         cameraPosition -= glm::normalize(glm::cross(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw))), glm::vec3(0.0f, 1.0f, 0.0f))) * cameraSpeed;
//     } else if (key == 'd') {
//         cameraPosition += glm::normalize(glm::cross(glm::vec3(glm::sin(glm::radians(cameraYaw)), 0.0f, glm::cos(glm::radians(cameraYaw))), glm::vec3(0.0f, 1.0f, 0.0f))) * cameraSpeed;
//     }
//     // Moving up (Q) and down (E)
//     else if (key == 'q') {
//         cameraPosition.y += cameraSpeed;
//     } else if (key == 'e') {
//         cameraPosition.y -= cameraSpeed;
//     }
//     updateViewMatrix();  // Update view matrix after moving camera
//     glutPostRedisplay();  // Request to update the display
// }

// // Handle camera rotation using arrow keys
// void specialKeys(int key, int x, int y) {
//     if (key == GLUT_KEY_UP) {
//         cameraPitch += cameraRotationSpeed;
//     } else if (key == GLUT_KEY_DOWN) {
//         cameraPitch -= cameraRotationSpeed;
//     } else if (key == GLUT_KEY_LEFT) {
//         cameraYaw -= cameraRotationSpeed;
//     } else if (key == GLUT_KEY_RIGHT) {
//         cameraYaw += cameraRotationSpeed;
//     }
//     updateViewMatrix();  // Update view matrix after rotating the camera
//     glutPostRedisplay();  // Request to update the display
// }

// // Render the box
// void renderBox(const RigidBody& body) {
//     // Get position and transformation matrix from the RigidBody
//     glm::vec3 position = glm::vec3(body.getPosition().getX(), body.getPosition().getY(), body.getPosition().getZ());
//     glm::mat4 transformationMatrix = body.getTransformMatrix().toGlm(); // Assuming you have a toGlm() method

//     glPushMatrix();

//     // Apply the transformation matrix (translation + rotation + scaling)
//     glMultMatrixf(glm::value_ptr(transformationMatrix));

//     // Draw the cube (static geometry with dynamic transformations)
//     glBegin(GL_QUADS);

//     // Front face (Red)
//     glColor3f(1.0f, 0.0f, 0.0f);
//     glVertex3f(-0.5f, -0.5f, 0.5f);
//     glVertex3f( 0.5f, -0.5f, 0.5f);
//     glVertex3f( 0.5f,  0.5f, 0.5f);
//     glVertex3f(-0.5f,  0.5f, 0.5f);

//     // Back face (Green)
//     glColor3f(0.0f, 1.0f, 0.0f);
//     glVertex3f(-0.5f, -0.5f, -0.5f);
//     glVertex3f(-0.5f,  0.5f, -0.5f);
//     glVertex3f( 0.5f,  0.5f, -0.5f);
//     glVertex3f( 0.5f, -0.5f, -0.5f);

//     // Top face (Blue)
//     glColor3f(0.0f, 0.0f, 1.0f);
//     glVertex3f(-0.5f,  0.5f,  0.5f);
//     glVertex3f( 0.5f,  0.5f,  0.5f);
//     glVertex3f( 0.5f,  0.5f, -0.5f);
//     glVertex3f(-0.5f,  0.5f, -0.5f);

//     // Bottom face (Yellow)
//     glColor3f(1.0f, 1.0f, 0.0f);
//     glVertex3f(-0.5f, -0.5f,  0.5f);
//     glVertex3f(-0.5f, -0.5f, -0.5f);
//     glVertex3f( 0.5f, -0.5f, -0.5f);
//     glVertex3f( 0.5f, -0.5f,  0.5f);

//     // Right face (Magenta)
//     glColor3f(1.0f, 0.0f, 1.0f);
//     glVertex3f( 0.5f, -0.5f,  0.5f);
//     glVertex3f( 0.5f, -0.5f, -0.5f);
//     glVertex3f( 0.5f,  0.5f, -0.5f);
//     glVertex3f( 0.5f,  0.5f,  0.5f);

//     // Left face (Cyan)
//     glColor3f(0.0f, 1.0f, 1.0f);
//     glVertex3f(-0.5f, -0.5f,  0.5f);
//     glVertex3f(-0.5f,  0.5f,  0.5f);
//     glVertex3f(-0.5f,  0.5f, -0.5f);
//     glVertex3f(-0.5f, -0.5f, -0.5f);

//     glEnd();

//     glPopMatrix();
// }

// // Display callback
// void display() {
//     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//     // Set the camera matrix (view matrix)
//     glMatrixMode(GL_MODELVIEW);
//     glLoadMatrixf(glm::value_ptr(viewMatrix));

//     // Render the box
//     // renderBox(boxBody);
//     for (auto& body : rigidBodies) {
//         Vector3 g = Vector3(0.0f, -9.8f, 0.0f);

//         body.applyForce(g, body.getPosition());

//         renderBox(body);
//     }

//     // Swap buffers
//     glutSwapBuffers();
// }

// // Update function to simulate physics
// void update(int value) {
//     // boxBody.applyForce(Vector3(0.0f, -50.0f, 0.0f), Vector3(0.0f, 0.3f, 0.3f));
//     // boxBody.applyForce(Vector3(100.0f, 100.0f, 0.0f), Vector3(0.0f, 0.3f, 0.3f));

//     // Integrate physics
//     boxBody.integratePhysics(0.016f);

//     rigidBodies[0].applyForce(Vector3(2.0f, 2.0f, 0.0f), rigidBodies[0].getPosition());
//     rigidBodies[1].applyForce(Vector3(1.0f, 0.0f, -3.0f), rigidBodies[1].getPosition());
//     rigidBodies[2].applyForce(Vector3(-1.0f, 0.0f, 5.0f), rigidBodies[2].getPosition());
//     rigidBodies[3].applyForce(Vector3(-2.0f, 0.0f, 0.0f), rigidBodies[3].getPosition());
//     rigidBodies[4].applyForce(Vector3(5.0f, -8.0f, 0.0f), rigidBodies[4].getPosition());
//     for (auto& body : rigidBodies) {
//         // body.applyForce(Vector3(10.0f, 0.0f, 0.0f), body.getPosition());
//         body.integratePhysics(0.1f);
//     }


//     aabb1.minExt = rigidBodies[0].getPosition() - Vector3(1, 1, 1);
//     aabb1.maxExt = rigidBodies[0].getPosition() + Vector3(1, 1, 1);

//     aabb2.minExt = rigidBodies[1].getPosition() - Vector3(1, 1, 1);
//     aabb2.maxExt = rigidBodies[1].getPosition() + Vector3(1, 1, 1);

//     aabb3.minExt = rigidBodies[2].getPosition() - Vector3(1, 1, 1);
//     aabb3.maxExt = rigidBodies[2].getPosition() + Vector3(1, 1, 1);

//     aabb4.minExt = rigidBodies[3].getPosition() - Vector3(1, 1, 1);
//     aabb4.maxExt = rigidBodies[3].getPosition() + Vector3(1, 1, 1);

//     aabb5.minExt = rigidBodies[4].getPosition() - Vector3(1, 1, 1);
//     aabb5.maxExt = rigidBodies[4].getPosition() + Vector3(1, 1, 1);


//     // Update AABBs (checking collisions)
//     // AABBTree broadphase;
//     for (auto& body : rigidBodies) {
//         AABB aabb(Vector3(body.getPosition().getX(), body.getPosition().getY(), body.getPosition().getZ()), Vector3(1.0f, 1.0f, 1.0f));
//         broadphase.Add(&aabb);
//     }

//     // Check for collisions and print them
//     broadphase.Update();
//     ColliderPairList& collidingPairs = broadphase.ComputeCollidingPairs();

//     if (!collidingPairs.empty()) {
//         for (const auto& pair : collidingPairs) {
//             std::cout << "Collision detected between " << pair.first->getRigidBody()->getName() << " and "
//                       << pair.second->getRigidBody()->getName() << std::endl;
//         }
//     }

//     // Request next frame update
//     glutTimerFunc(16, update, 0);
//     glutPostRedisplay();
// }

// // Main function
// int main(int argc, char** argv) {
//     if (!initGL(argc, argv, 800, 600)) {
//         return -1;
//     }

//     // // Create the box shape and rigid body
//     // BoxShape box = BoxShape(10.0f, 10.0f, 10.0f, 1.0f);  // 1x1x1 box
//     // boxBody = RigidBody(box, Vector3(0.0f, 0.0f, 0.0f));  // Start position at (0, 0, -10)

//     // // Apply some force to the box body for testing
//     // boxBody.applyForce(Vector3(100.0f, 100.0f, 0.0f), Vector3(0.0f, 0.3f, 0.3f));


//     // Create 5 rigid bodies
//     rigidBodies.push_back(RigidBody(BoxShape(10.0f, 10.0f, 10.0f, 1.0f), Vector3(0, 0, 0), Vector3(1, 0, 0), "Box 1", Vector3(10, 10, 0), Vector3(10, -10, 0)));
//     rigidBodies.push_back(RigidBody(BoxShape(10.0f, 10.0f, 10.0f, 1.0f), Vector3(2, 0, 0), Vector3(0, 1, 0), "Box 2"));
//     rigidBodies.push_back(RigidBody(BoxShape(10.0f, 10.0f, 10.0f, 1.0f), Vector3(4, 0, 0), Vector3(0, 0, 1), "Box 3"));
//     rigidBodies.push_back(RigidBody(BoxShape(10.0f, 10.0f, 10.0f, 1.0f), Vector3(6, 0, 0), Vector3(-1, 0, 0), "Box 4"));
//     rigidBodies.push_back(RigidBody(BoxShape(10.0f, 10.0f, 10.0f, 1.0f), Vector3(8, 0, 0), Vector3(0, -1, 0), "Box 5"));

// // Create corresponding AABBs for each rigid body
//     // AABB aabb1(Vector3(0, 0, 0), Vector3(1, 1, 1));
//     // AABB aabb2(Vector3(2, 0, 0), Vector3(3, 1, 1));
//     // AABB aabb3(Vector3(4, 0, 0), Vector3(5, 1, 1));
//     // AABB aabb4(Vector3(6, 0, 0), Vector3(7, 1, 1));
//     // AABB aabb5(Vector3(8, 0, 0), Vector3(9, 1, 1));

//     aabb1 = AABB(Vector3(0, 0, 0), Vector3(1, 1, 1));
//     aabb2 = AABB(Vector3(2, 0, 0), Vector3(3, 1, 1));
//     aabb3 = AABB(Vector3(4, 0, 0), Vector3(5, 1, 1));
//     aabb4 = AABB(Vector3(6, 0, 0), Vector3(7, 1, 1));
//     aabb5 = AABB(Vector3(8, 0, 0), Vector3(9, 1, 1));

//     Cube sphere1(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.6f, 1.0f));
//     Cube sphere2(Vector3(2.0f, 0.0f, 0.0f), Vector3(1.3f, 1.0f, 1.0f));
//     Cube sphere3(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.6f, 1.0f));
//     Cube sphere4(Vector3(2.0f, 0.0f, 0.0f), Vector3(1.3f, 1.0f, 1.0f));
//     Cube sphere5(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.6f, 1.0f));

//     // Link AABBs to the rigid bodies (Collider)
//     aabb1.collider = new Collider(1.0f, Matrix4(), Vector3(), &rigidBodies[0], &sphere1);
//     aabb2.collider = new Collider(1.0f, Matrix4(), Vector3(), &rigidBodies[1], &sphere2);
//     aabb3.collider = new Collider(1.0f, Matrix4(), Vector3(), &rigidBodies[2], &sphere3);
//     aabb4.collider = new Collider(1.0f, Matrix4(), Vector3(), &rigidBodies[3], &sphere4);
//     aabb5.collider = new Collider(1.0f, Matrix4(), Vector3(), &rigidBodies[4], &sphere5);

//     // Create an AABBTree instance for broad-phase collision detection
//     // AABBTree broadphase;
//     broadphase.Add(&aabb1);
//     broadphase.Add(&aabb2);
//     broadphase.Add(&aabb3);
//     broadphase.Add(&aabb4);
//     broadphase.Add(&aabb5);

//     // Set GLUT callbacks
//     glutDisplayFunc(display);
//     glutKeyboardFunc(keyboard);
//     glutSpecialFunc(specialKeys);
//     glutTimerFunc(1000 / 60, update, 0);

//     // Start GLUT main loop
//     glutMainLoop();


//     delete aabb1.collider;
//     delete aabb2.collider;
//     delete aabb3.collider;
//     delete aabb4.collider;
//     delete aabb5.collider;

//     return 0;
// }

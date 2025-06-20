#include "App.h"
#include "../core/Vector3.h"
#include "../core/Matrix4.h"
#include "../core/Quaternion.h"
#include "../core/RigidBody.h"
#include "../core/AABB.h"
#include "../core/Collider.h"
#include "../core/shape/Shape.h"
#include "../core/PhysicsWorld.h"

#include <memory>

int main() {
    App app;
    app.run();

    // Cube cube1(Vector3(2, 3, 1));

    // RigidBody rb1(cube1, Vector3(0,0,0), Vector3(1,0.5, 0), "cube 1");

    // Collider cl1(1.0f, Matrix4(), Vector3(), &rb1, &cube1, Collider::ColliderType::DYNAMIC);

    // AABB aabb1 = AABB::fromHalfCentralExtents(rb1.getPosition(), Vector3(2,3,1));

    // PhysicsWorld physicsWorld;

    // std::shared_ptr<RigidBody> rb1ptr= &rb1;
    // std::shared_ptr<Collider> cl1ptr = &cl1;

    // PhysicsWorld.addRigidBody(rb1ptr, cl1ptr);

    // PhysicsWorld.simulate();

    return 0;
}
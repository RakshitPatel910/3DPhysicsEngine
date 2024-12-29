#pragma once

#include "Vector3.h"
#include "Collider.h"
#include "shape/Shape.h"
#include "../utils/MinowskiDifference.h"
#include "Simplex.h"

class GJK
{
public:
    bool isIntersecting(const Shape& shapeA, const Shape& shapeB){
        Simplex simplex;

        Vector3 direction = Vector3(1, 0, 0); // initial direction

        Vector3 support = shapeA->Support(direction) - shapeB.Support(-direction);
        simplex.addPoint(support);

        direction = -direction;

        while (true) {
            support = shapeA->Support(direction) - shapeB.Support(-direction);

            if(support.dot(direction) <= 0){ // no collision
                return false;
            }

            simplex.addPoint(support);

            if(simplex.containsOrigin(direction)){ // collision
                return true;
            }
        }
        
    }
};
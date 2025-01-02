#pragma once

#include "Vector3.h"
#include "Collider.h"
#include "shape/Shape.h"
#include "../utils/MinowskiDifference.h"
#include "Simplex.h"

class GJK
{
private:
    unsigned leastSignificantComponent(const Vector3& v){
        float x = std::abs(v.getX());
        float y = std::abs(v.getY());
        float z = std::abs(v.getZ());

        if(x < y){
            return x < z ? 0 : 2; // 0 ==> x is least significant, 2 ==> z is least significant
        }
        else {
            return y < z ? 1 : 2; // 1 ==> y is least significant
        }
    }

    void fixTetrahedronWinding(Simplex& simplex){
        const Vector3 v30 = simplex[0] - simplex[3];
        const Vector3 v31 = simplex[1] - simplex[3];
        const Vector3 v32 = simplex[2] - simplex[3];

        const float det = v30.dot(v31.cross(v32)); // determinant

        if( det > 0.0f ){
            std:swap(simplex[0], simplex[1]);
        }
    }

public:
    static constexpr k_eps = 0.00001f;
    static constexpr k_epsSq = k_eps * k_eps;
    // const float PI = 3.14159265358979323846f; 
    // const float PI_2 = PI / 2.0f;             
    // const float PI_3 = PI / 3.0f;             


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

    void simplexToTetrahedron(Simplex& simplex, Simplex& simplexA, Simplex& simplexB, const Shape& shapeA, const Shape& shapeB){
        static const Vector3 k_dir[] = {
            Vector3(1.0f, 0.0f, 0.0f),
            Vector3(-1.0f, 0.0f, 0.0f),
            Vector3(0.0f, 1.0f, 0.0f),
            Vector3(0.0f, -1.0f, 0.0f),
            Vector3(0.0f, 0.0f, 1.0f),
            Vector3(0.0f, 0.0f, -1.0f)
        };

        static const Vector3 k_axes[] = {
            Vector3(1.0f, 0.0f, 0.0f),
            Vector3(0.0f, 1.0f, 0.0f),
            Vector3(0.0f, 0.0f, 1.0f)
        };

        switch (simplex.size())
        {
            case 1:
                for( const Vector3& dir : k_dir ){
                    simplex[1] = shapeA->Support(dir) - shapeB->Support(-dir);

                    if( (simplex[1] - simplex[0]).lengthSq >= k_epsSq ){ // if far enough
                        break;
                    }
                }
            
            case 2:
                Vector3 line = simplex[1] - simplex[0];
                unsigned leastSigAx = leastSignificantComponent(line); // axis with smallest component

                Vector3 dir = line.cros(k_axes[leastSigAx]);

                Matrix4 rMat = Matrix4::getRotationMatrixByAngleOnAxis(60, line); // matrix to rotate by 60

                for( int i = 0; i < 6; i++ ){
                    simplex[2] = shapeA->Support(dir) - shapeB->Support(-dir);

                    if( simplex[2].lengthSq >= k_epsSq ){
                        break;
                    }         

                    dir = rMat.transformVec(line);       
                }
            
            case 3:
                const Vector3 v01 = simplex[1] - simplex[0]; 
                const Vector3 v02 = simplex[2] - simplex[0]; 

                Vector3 dir = v01.cross(v02);

                simplex[3] = shapeA->Support(dir) - shapeB->Support(-dir);

                if( simplex[3].lengthSq < k_epsSq ){ // if not far enough
                    dir.negate();
                    simplex[3] = shapeA->Support(dir) - shapeB->Support(-dir);
                }
        }

        fixTetrahedronWinding(simplex&); // so that all normal of triangle face outwards of tetrahedron

    }
    
};
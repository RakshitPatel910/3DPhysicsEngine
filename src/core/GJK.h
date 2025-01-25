#pragma once

#include<algorithm>

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
            std::swap(simplex[0], simplex[1]);
        }
    }

public:
    static constexpr float k_eps = 0.00001f;
    static constexpr float k_epsSq = k_eps * k_eps;
    // const float PI = 3.14159265358979323846f; 
    // const float PI_2 = PI / 2.0f;             
    // const float PI_3 = PI / 3.0f;             


    // bool isIntersecting(Simplex& simplex, const Shape& shapeA, const Shape& shapeB, Collider& colA, Collider& colB){
    bool isIntersecting(Simplex& simplex, Collider& colA, Collider& colB){
        // Simplex simplex;

        Vector3 direction = Vector3(1, 0, 0); // initial direction

        Vector3 support = colA.Support(direction) - colB.Support(-direction);

        if(direction.dot(support) >= support.length() * 0.8f){ // stability check
            direction = Vector3(0, 1, 0);
            support = colA.Support(direction) - colB.Support(-direction);
        }
        
        simplex.addPoint(support);

        direction = -direction;

        while (true) {
            support = colA.Support(direction) - colB.Support(-direction);

            if(support.dot(direction) <= 0.0f){ // no collision
                return false;
            }

            simplex.addPoint(support);

            if(simplex.containsOrigin(direction)){ // collision
                // simplexToTetrahedron(simplex, shapeA, shapeB, colA, colB);
                simplexToTetrahedron(simplex, colA, colB);

                return true;
            }
        }
        
    }

    // void simplexToTetrahedron(Simplex& simplex, Simplex& simplexA, Simplex& simplexB, const Shape& shapeA, const Shape& shapeB){
    // void simplexToTetrahedron(Simplex& simplex, const Shape& shapeA, const Shape& shapeB, Collider& colA, Collider& colB){
    void simplexToTetrahedron(Simplex& simplex, Collider& colA, Collider& colB){
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

        Vector3 dir;
        Vector3 line;
        unsigned leastSigAx;
        Matrix4 rMat;

        switch (simplex.size())
        {
            case 1:
                for( const Vector3& dir : k_dir ){
                    simplex[1] = colA.Support(dir) - colB.Support(-dir);

                    if( (simplex[1] - simplex[0]).lengthSq() >= k_epsSq ){ // if far enough
                        break;
                    }
                }
                [[fallthrough]];
            case 2:
                line = simplex[1] - simplex[0];
                leastSigAx = leastSignificantComponent(line); // axis with smallest component

                // Vector3 dir = line.cross(k_axes[leastSigAx]);
                dir = line.cross(k_axes[leastSigAx]);

                rMat = Matrix4::getRotationMatrixByAngleOnAxis(60, line); // matrix to rotate by 60

                for( int i = 0; i < 6; i++ ){
                    simplex[2] = colA.Support(dir) - colB.Support(-dir);

                    if( simplex[2].lengthSq() >= k_epsSq ){
                        break;
                    }         

                    dir = rMat.transformVec(line);       
                }
                [[fallthrough]];

            case 3:
                const Vector3 v01 = simplex[1] - simplex[0]; 
                const Vector3 v02 = simplex[2] - simplex[0]; 

                // Vector3 dir = v01.cross(v02);
                dir = v01.cross(v02);

                simplex[3] = colA.Support(dir) - colB.Support(-dir);

                if( simplex[3].lengthSq() < k_epsSq ){ // if not far enough
                    dir.negate();
                    simplex[3] = colA.Support(dir) - colB.Support(-dir);
                }
        }

        fixTetrahedronWinding(simplex); // so that all normal of triangle face outwards of tetrahedron

    }
    
};
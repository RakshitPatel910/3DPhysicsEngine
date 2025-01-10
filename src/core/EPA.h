#pragma once

#include<vector>
#include<set>
#include<cfloat>
#include<algorithm>

#include "Vector3.h"
#include "Matrix4.h"
#include "Simplex.h"

struct Triangle
{
    Vector3 vertices[3];
    Vector3 normal;
    float dist; // distance between origin and triangle

    Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2) : vertices{v0, v1, v2} {
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;

        normal = e1.cross(e2).normalized();
        // normal = e1.cross(e2);
        dist = std::abs(normal.dot(v0));
    }
};

class Polytope
{
public:
    std::vector<Vector3> vertexList;
    std::vector<Triangle> triangleList;
    // std::vector<std::pair<int, int>> edgeList;
    std::vector<std::pair<Vector3 , Vector3 >> edgeList;

    void clear(){
        vertexList.clear();
        triangleList.clear();
        edgeList.clear();
    }
};

struct ContactData
{
    Vector3 worldContactPointA;
    Vector3 worldContactPointB;

    // Vector3 localContactPointA;
    // Vector3 localContactPointB;

    Vector3 contactNormal;
    Vector3 contactTangent1;
    Vector3 contactTangent2;

    float penetrationDepth;

    float normalImpulseSum;
    float tangent1ImpulseSum;
    float tangent2ImpulseSum;

    ContactData() : normalImpulseSum(0.0f), tangent1ImpulseSum(0.0f), tangent2ImpulseSum(0.0f) {}
};

class EPA
{
// private:

public:
    Polytope polytope;
    const float k_eps = 0.00001f;
    static const unsigned max_iter = 50;

    bool IsDegenerate(const Triangle& triangle) {
        Vector3 e1 = triangle.vertices[1] - triangle.vertices[0]; // edge 1
        Vector3 e2 = triangle.vertices[2] - triangle.vertices[0]; // edge 2

        // Calculate cross product magnitude
        float crossProductMagnitude = e1.cross(e2).length();

        // If the cross product magnitude is smaller than epsilon, it's degenerate
        // return crossProductMagnitude < k_eps;
        return crossProductMagnitude < k_eps || std::abs(triangle.dist) < k_eps;
    }

    void UpdatePolytope(const Vector3& supportPoint){
        // adding supportPoint to vertexList
        bool pointExists = false;
        for (const Vector3& v : polytope.vertexList) {
            if ( v == supportPoint ) {
                pointExists = true;
                break;
            }
        }
        if (!pointExists) {
            polytope.vertexList.push_back(supportPoint);
        }

        polytope.edgeList.clear();

        for( size_t i = 0; i < polytope.triangleList.size(); ){
            Triangle& triangle = polytope.triangleList[i];

            // Skip degenerate triangles
            if (IsDegenerate(triangle)) {
                polytope.triangleList.erase(polytope.triangleList.begin() + i); // Remove degenerate triangle
                // i--; // Adjust the index as the list shrinks
                
                continue;
            }

            // adding all edges of visible triangle;
            if(triangle.normal.dot(supportPoint - triangle.vertices[0]) > 0.0f){ // normal of triangle points towards supportPoint (i.e. triangle visible from supportPoint)
                polytope.edgeList.emplace_back(triangle.vertices[0], triangle.vertices[1]);
                polytope.edgeList.emplace_back(triangle.vertices[1], triangle.vertices[2]);
                polytope.edgeList.emplace_back(triangle.vertices[2], triangle.vertices[0]);

                polytope.triangleList.erase(polytope.triangleList.begin() + i);
                continue;
            }
            i++;
        }

        // removing reoccuring edges, (a, b) and (b, a) both exist ==> remove both
        for( size_t i = 0; i < polytope.edgeList.size(); i++ ){
            auto [a, b] = polytope.edgeList[i];
            auto it = std::find(polytope.edgeList.begin(), polytope.edgeList.end(), std::make_pair(b, a));

            if( it != polytope.edgeList.end() ){
                polytope.edgeList.erase(it);
                polytope.edgeList.erase(polytope.edgeList.begin() + i);

                i--;
            }
        }

        for( const auto& [a, b] : polytope.edgeList){

            // Triangle newTriangle = Triangle(a, b, polytope.vertexList[polytope.edgeList.size()]);
            Triangle newTriangle = Triangle(polytope.vertexList.back(), a, b);
            polytope.triangleList.push_back(newTriangle);
        }

        polytope.edgeList.clear();
    }

    ContactData RunEPA(const Simplex& simplex, const Shape& shapeA, const Shape& shapeB){
        polytope.clear();
        polytope.vertexList.assign(simplex.getSimplex().begin(), simplex.getSimplex().begin() + 4);

        polytope.triangleList.push_back(Triangle(simplex[0], simplex[1], simplex[2]));
        polytope.triangleList.push_back(Triangle(simplex[0], simplex[2], simplex[3]));
        polytope.triangleList.push_back(Triangle(simplex[0], simplex[3], simplex[1]));
        polytope.triangleList.push_back(Triangle(simplex[1], simplex[3], simplex[2]));

        unsigned iter = 0;
        while (true) {
        // while (iter++ < max_iter) {

            // if (iter++ >= max_iter){
            //     return false;
            // }

            float minDist = FLT_MAX;
            Triangle* closestTriangle = nullptr;

            // finding closest triangle to origin
            for( Triangle& triangle : polytope.triangleList ){
                if( triangle.dist < minDist ){
                    closestTriangle = &triangle;
                    minDist = triangle.dist;
                }
            }

            Vector3 supportPoint = shapeA.Support(closestTriangle->normal) - shapeB.Support(-(closestTriangle->normal));

            if( (closestTriangle->normal).dot(supportPoint) < k_eps || iter++ >= max_iter ){
                ContactData contactData;

                contactData.worldContactPointA = supportPoint;
                contactData.worldContactPointB = supportPoint;

                Vector3 zeroVec = Vector3();
                if( closestTriangle->normal == zeroVec ){
                    contactData.contactNormal = supportPoint.normalized();
                }
                else {
                    contactData.contactNormal = closestTriangle->normal;
                }

                if (contactData.contactNormal.getX() >= 0.57735f) {
                    contactData.contactTangent1 = Vector3(contactData.contactNormal.getY(), -contactData.contactNormal.getX(), 0.0f).normalized();
                }
                else {
                    contactData.contactTangent1 = Vector3(0.0f, contactData.contactNormal.getZ(), -contactData.contactNormal.getY()).normalized();
                }
                contactData.contactTangent2 = contactData.contactNormal.cross(contactData.contactTangent1);

                contactData.penetrationDepth = minDist;

                return contactData;
            }

            UpdatePolytope(supportPoint);
        }
        
    }
};


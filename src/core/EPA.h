#pragma once

#include<vector>
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

        normal = e1.cross(e2).normalize();
        distance = normal.dot(v0);
    }
};

class Polytope
{
public:
    std::vector<Vector3> vertexList;
    std::vector<Triangle> triangleList;
    std::vector<std::pair<int, int>> edgeList;

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
};

class EPA
{
// private:

public:
    Polytope polytope;

    void UpdatePolytope(const Vector3& supportPoint){
        polytope.edgeList.clear();

        // finding all visible triangles
        for( size_t i = 0; i < polytope.triangleList.size(); i++ ){
            Triangle& triangle = polytope.triangleList[i];

            // adding all edges of visible triangle;
            if(triangle.normal.dot(support - triangle.vertices[0]) > 0.0f){ // normal of triangle points towards supportPoint (i.e. triangle visible from supportPoint)
                polytope.edgeList.emplace_back(0, 1);
                polytope.edgeList.emplace_back(0, 2);
                polytope.edgeList.emplace_back(2, 1);

                polytope.triangleList.erase(polytope.triangleList.begin() + i);
                i--;
            }    
        }

        // removing reoccuring edges, (a, b) and (b, a) both exist ==> remove both
        if( size_t i = 0; i < polytope.edgeList.size(); i++){
            auto [a, b] = polytope.edgeList[i];
            auto it = std::find(polytope.edgeList.begin(), polytope.edgeList.end(), std::make_pair(b, a));

            if( it != polytope.edgeList.end() ){
                polytope.edgeList.erase(it);
                polytope.edgeList.erase(edgeList.begin() + i);

                i--;
            }
        }

        // adding new triangles
        for( const auto& [a, b] : polytope.edgeList){
            Triangle newTriangle = Triangle(polytope.vertexList[a], polytope.vertexList[b], supportPoint);
            polytope.triangleList.push_back(newTriangle);
        }

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
    }

    ContactData RunEPA(const Simplex& simplex, const Shape& shapeA, const Shape& shapeB){
        polytope.clear();
        polytope.vertexList.assign(simplex.m_simplex, simplex.m_simplex + 4);

        polytope.triangleList.push_back(Triangle(simplex[0], simplex[1], simplex[2]));
        polytope.triangleList.push_back(Triangle(simplex[0], simplex[2], simplex[3]));
        polytope.triangleList.push_back(Triangle(simplex[0], simplex[3], simplex[1]));
        polytope.triangleList.push_back(Triangle(simplex[1], simplex[3], simplex[2]));

        while (true) {
            float minDist = FLT_MAX;
            Triangle* closestTriangle = nullptr;

            // finding closest triangle to origin
            for( Triangle& triangle : polytope.triangleList ){
                if( triangle.dist < minDist ){
                    closestTriangle = &triangle;
                    minDist = triangle.dist;
                }
            }

            Vector3 supportPoint = shapeA->Support(closestTriangle->normal) - shapeB->Support(-(closestTriangle->normal));

            if( (closestTriangle->normal).dot(supportPoint) < k_eps ){
                ContactData contactData;

                contactData.worldContactPointA = supportPoint;
                contactData.worldContactPointB = supportPoint;

                contactData.contactNormal = closestTriangle->normal;

                if (contactNormal.getX() >= 0.57735f) {
                    contactData.contactTangent1 = Vector3(contactNormal.getY(), -contactNormal.getX(), 0.0f).normalized();
                }
                else {
                    contactData.contactTangent1 = Vector3(0.0f, contactNormal.getZ(), -contactNormal.getY()).normalized();
                }
                contactData.contactTangent2 = contactNormal.cross(contactTangent1);

                contactData.penetrationDepth = minDist;

                return contactData;
            }

            UpdatePolytope(supportPoint);
        }
        
    }
};


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

    Vector3 localContactPointA;
    Vector3 localContactPointB;

    Vector3 contactNormal;
    Vector3 contactTangent1;
    Vector3 contactTangent2;

    float penetrationDepth;

    float normalImpulseSum;
    float tangent1ImpulseSum;
    float tangent2ImpulseSum;

    bool isPersistent;

    ContactData() : normalImpulseSum(0.0f), tangent1ImpulseSum(0.0f), tangent2ImpulseSum(0.0f) 
    { 
        isPersistent = false; 
    }
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

    // void UpdatePolytope(const Vector3& supportPoint){
    //     // adding supportPoint to vertexList
    //     bool pointExists = false;
    //     for (const Vector3& v : polytope.vertexList) {
    //         if ( v == supportPoint ) {
    //             pointExists = true;
    //             break;
    //         }
    //     }
    //     if (!pointExists) {
    //         polytope.vertexList.push_back(supportPoint);
    //     }

    //     polytope.edgeList.clear();

    //     for( size_t i = 0; i < polytope.triangleList.size(); ){
    //         Triangle& triangle = polytope.triangleList[i];

    //         // Skip degenerate triangles
    //         if (IsDegenerate(triangle)) {
    //             polytope.triangleList.erase(polytope.triangleList.begin() + i); // Remove degenerate triangle
    //             // i--; // Adjust the index as the list shrinks
                
    //             continue;
    //         }

    //         // adding all edges of visible triangle;
    //         if(triangle.normal.dot(supportPoint - triangle.vertices[0]) > 0.0f){ // normal of triangle points towards supportPoint (i.e. triangle visible from supportPoint)
    //             polytope.edgeList.emplace_back(triangle.vertices[0], triangle.vertices[1]);
    //             polytope.edgeList.emplace_back(triangle.vertices[1], triangle.vertices[2]);
    //             polytope.edgeList.emplace_back(triangle.vertices[2], triangle.vertices[0]);

    //             polytope.triangleList.erase(polytope.triangleList.begin() + i);
    //             continue;
    //         }
    //         i++;
    //     }

    //     // removing reoccuring edges, (a, b) and (b, a) both exist ==> remove both
    //     for( size_t i = 0; i < polytope.edgeList.size(); i++ ){
    //         auto [a, b] = polytope.edgeList[i];
    //         auto it = std::find(polytope.edgeList.begin(), polytope.edgeList.end(), std::make_pair(b, a));

    //         if( it != polytope.edgeList.end() ){
    //             polytope.edgeList.erase(it);
    //             polytope.edgeList.erase(polytope.edgeList.begin() + i);

    //             i--;
    //         }
    //     }

    //     for( const auto& [a, b] : polytope.edgeList){

    //         // Triangle newTriangle = Triangle(a, b, polytope.vertexList[polytope.edgeList.size()]);
    //         Triangle newTriangle = Triangle(polytope.vertexList.back(), a, b);
    //         polytope.triangleList.push_back(newTriangle);
    //     }

    //     polytope.edgeList.clear();
    // }

    void AddEdge2(std::vector<std::pair<Vector3, Vector3>>& aEdgeList, const Vector3& a, const Vector3& b)
    {
        for (auto iterator = aEdgeList.begin(); iterator != aEdgeList.end(); ++iterator)
        {
            if (iterator->first == b && iterator->second == a)
            {
                // Encountered the same edge with opposite winding, remove it and don't add a new one
                aEdgeList.erase(iterator);
                return;
            }
        }
        aEdgeList.push_back({a, b});
    }

    inline void BarycentricProjection(const Vector3& aPoint, Vector3& a, Vector3& b, Vector3& c, float& u, float& v, float& w)
    {
        glm::vec3 v0 = b - a, v1 = c - a, v2 = aPoint - a;
        float d00 = glm::dot(v0, v0);
        float d01 = glm::dot(v0, v1);
        float d11 = glm::dot(v1, v1);
        float d20 = glm::dot(v2, v0);
        float d21 = glm::dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
        u = 1.0f - v - w;
    }

    // Correct update polytop function
    void UpdatePolytope(const Vector3& supportPoint) {

        for(auto it = polytope.triangleList.begin(); it != polytope.triangleList.end(); ) {

            Vector3 planeVec = supportPoint - it->vertices[0];

            if( it->normal.dot(planeVec) > 0.0f ){
                AddEdge2(polytope.edgeList, it->vertices[0], it->vertices[1]);
                AddEdge2(polytope.edgeList, it->vertices[1], it->vertices[2]);
                AddEdge2(polytope.edgeList, it->vertices[2], it->vertices[0]);

                it = polytope.triangleList.erase(it);
                continue;
            }
            it++;
        }

        for(auto it = polytope.edgeList.begin(); it != polytope.edgeList.end(); it++) {
            polytope.triangleList.emplace_back(Triangle(supportPoint, it->first, it->second));

            Triangle newTriangle = Triangle(supportPoint, it->first, it->second);
        }

        polytope.edgeList.clear();
    }

    // ContactData RunEPA(const Simplex& simplex, const Shape& shapeA, const Shape& shapeB, Collider& colA, Collider& colB){
    ContactData RunEPA(const Simplex& simplex, Collider& colA, Collider& colB){
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
                if( std::fabs(triangle.dist) < minDist ){
                    closestTriangle = &triangle;
                    minDist = triangle.dist;
                }
            }

            // Vector3 supportPoint = shapeA.Support(closestTriangle->normal) - shapeB.Support(-(closestTriangle->normal));
            Vector3 supportPoint = colA.Support(closestTriangle->normal) - colB.Support(closestTriangle->normal * -1);

            if( std::fabs((closestTriangle->normal).dot(supportPoint) - minDist) < k_eps || iter++ >= max_iter ){
                ContactData contactData;

                const float distanceFromOrigin = (closestTriangle->normal).dot(closestTriangle->vertices[0]);

	            float bary_u, bary_v, bary_w;

                BarycentricProjection(closestTriangle->normal * distanceFromOrigin, closestTriangle->vertices[0], closestTriangle->vertices[1], closestTriangle->vertices[2], bary_u, bary_v, bary_w);

                // if (fabs(bary_u) > 1.0f || fabs(bary_v) > 1.0f || fabs(bary_w) > 1.0f)
                //     return false;
                // if (!IsValid(bary_u) || !IsValid(bary_v) || !IsValid(bary_w))
                //     return false;

                Vector3 supportLocal1 = closestTriangle->vertices[0];
                Vector3 supportLocal2 = closestTriangle->vertices[1];
                Vector3 supportLocal3 = closestTriangle->vertices[2];

                contactData.worldContactPointA =  (supportLocal1 * bary_u) + (supportLocal2 * bary_v) + (supportLocal3 * bary_w);
                contactData.worldContactPointB =  (supportLocal1 * bary_u) + (supportLocal2 * bary_v) + (supportLocal3 * bary_w);

                contactData.contactNormal = (closestTriangle->normal).normalized();
                // contactData.contactNormal = closestTriangle->normal;

                if (contactData.contactNormal.getX() >= 0.57735f) {
                    contactData.contactTangent1 = Vector3(contactData.contactNormal.getY(), -contactData.contactNormal.getX(), 0.0f).normalized();
                }
                else {
                    contactData.contactTangent1 = Vector3(0.0f, contactData.contactNormal.getZ(), -contactData.contactNormal.getY()).normalized();
                }
                contactData.contactTangent2 = contactData.contactNormal.cross(contactData.contactTangent1);

                Vector3 Pa = colA.Support(contactData.contactNormal);
                Vector3 Pb = colB.Support(-contactData.contactNormal);

                // contactData.penetrationDepth = minDist;
                contactData.penetrationDepth = (std::abs(Pa.dot(contactData.contactNormal)) - std::abs(Pb.dot(contactData.contactNormal)));

                return contactData;
            }

            UpdatePolytope(supportPoint);
        }
        
    }
};


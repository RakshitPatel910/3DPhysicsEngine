#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "Collider.h"
#include "EPA.h"

class Manifold
{
public:
    const persistent_threshold = 0.001;

    Collider* colliderA;
    Collider* colliderB;
    
    std::vector<ContactData> persistentContacts;
    // size_t contactCount;
    size_t maxCount = 4;

    Manifold(Collider* colliderA, Collider* colliderB) : colliderA(colliderA), colliderB(colliderB) {}

    void updateManifold() {
        // identify persistent points
        for( ContactData& contact : persistentContacts ) {
            const Vector3 rAB = contact.worldContactPointA - contact.worldContactPointB;
            const Vector3 rA = contact.localContactPointA - contact.worldContactPointA;
            const Vector3 rB = contact.localContactPointB - contact.worldContactPointB;

            const isPenetrating = contact.normal.dot(rAB) <= 0.0f;

            const closeEnoughrA = rA.lengthSq() < persistent_threshold;
            const closeEnoughrB = rB.lengthSq() < persistent_threshold;

            // if( closeEnoughA && closeEnoughB ) {
            //     contact.isPersistent = true;
            // }
            // else {
            //     // Remove contact from Manifold
            // }

            contact.isPersistent = isPenetrating && closeEnoughrA && closeEnoughrB;
        }

        // remove non persistent points
        persistentContacts.erase(
            std::remove_if( 
                persistentContacts.begin(), 
                persistentContacts.end(), 
                [](const ContactData& contact){
                    return !contact.isPersistent;
                }
            ),
            persistentContacts.end()
        );

    }

    void addContactPoint(const ContactData& newContact){
        // proximity check for newPoint with existing manifold
        for( ContactData& contact : persistentContacts ) {
            const Vector3 rA = newContact.worldContactPointA - contact.worldContactPointA;
            const Vector3 rB = newContact.worldContactPointB - contact.worldContactPointB;

            const bool farEnoughrA = rA.lengthSq() > persistent_threshold;
            const bool farEnoughrB = rB.lengthSq() > persistent_threshold;

            if( !farEnoughrA && !farEnoughrB ) {
                // persistentContacts.push_back( newContact );
                return;
            }
        }

        persistentContacts.push_back( newContact );
        
        if( persistentContacts.size() > 4 ) {
            // find deepest penetrating contact point ===> [ 1st point of manifold ]
            ContactData* deepestPoint = nullptr;
            float maxPenetration = -std::numeric_limits<float>::max();

            for( ContactData& contact : persistentContacts ) {
                if( contact.penetrationDepth > maxPenetration ){
                    maxPenetration = contact.penetrationDepth;
                    deepestPoint = &contact;
                }
            }

            // find furthest point for 1st point ===> [ 2nd point of manifold ]
            ContactData* furthest1* = nullptr;
            float maxDist1 = -std::numeric_limits<float>::max();

            for( ContactData& contact : persistentContacts ) {
                float dist = (contact.worldContactPointA - deepest->worldContactPointA).lengthSq();

                if( dist > maxDist1 ){
                    maxDist1 = dist;
                    furthest1 = &contact;
                }
            }

            // find furthest from the line formed by the first two ===> [ 3rd point of manifold ]
            ContactData* furthest2 = nullptr;
            float maxDist2 = -std::numeric_limits<float>::max();

            for( ContactData& contact : persistentContacts ) {
                float dist = DistanceFromLineSegment(contact.worldContactPointA, deepest->worldContactPointA, furthest1->worldContactPointA);

                if( dist > maxDist2 ){
                    maxDist2 = dist;
                    furthest2 = &contact;
                }
            }

            // find furthest from the triangle formed by the first three ===> [ 4th point of manifold ]
            ContactData* furthest3 = nullptr;
            float maxDist3 = -std::numeric_limits<float>::max();

            for( ContactData& contact : persistentContacts ) {
                float dist = DistanceFromTriangle(contact.worldContactPointA, deepest->worldContactPointA, furthest1->worldContactPointA, furthest2->worldContactPointA);

                if( dist > maxDist3 ){
                    maxDist3 = dist; 
                    furthest3 = &contact;
                }
            }

            // rebuild manifold
            persistentContacts.clear();
            persistentContacts.push_back(*deepest);
            persistentContacts.push_back(*furthest1);
            persistentContacts.push_back(*furthest2);

            if (furthest3 && !isInsideTriangle(furthest3->worldContactPointA, 
                                            deepest->worldContactPointA, 
                                            furthest1->worldContactPointA, 
                                            furthest2->worldContactPointA)) {
                persistentContacts.push_back(*furthest3);
            }
        }
    }

private:
    float DistanceFromLineSegment(const Vector3& point, const Vector3& A, const Vector3& B) {
        const Vector3 AB = B - A;
        const Vector3 Ap = point - A;

        float t = Ap.dot(AB) / AB.dot(AB); // projection of Ap on infinite line AB
        t = std::max(0.0f, std::min(1.0f, t));

        const Vector3 closestPoint = A + AB * t;

        return (point - closestPoint).lengthSq();
    }

    bool IsInsideTriangle(const Vector3& point, const Vector3& A, const Vector3& B, const Vector3& C) {
        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = point - a;

        float d00 = v0.dot(v0);
        float d01 = v0.dot(v1);
        float d11 = v1.dot(v1);
        float d20 = v2.dot(v0);
        float d21 = v2.dot(v1);

        // Calculate the denominator of the barycentric coordinates
        float denom = d00 * d11 - d01 * d01;

        // Calculate the barycentric coordinates
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        // The point is inside the triangle if all barycentric coordinates are between 0 and 1
        return (u >= 0.0f && v >= 0.0f && w >= 0.0f);
    }

    float DistanceFromTriangle(const Vector3& point, const Vector3& A, const Vector3& B, const Vector3& C) {
        // Check if the point is inside the triangle
        if (isInsideTriangle(point, a, b, c)) {
            return 0.0f; // Point is inside the triangle, distance is zero
        }

        // Otherwise, calculate the distance to each of the three edges and return the minimum
        float distAB = distanceFromLineSegment(point, a, b); // Distance to edge AB
        float distBC = distanceFromLineSegment(point, b, c); // Distance to edge BC
        float distCA = distanceFromLineSegment(point, c, a); // Distance to edge CA

        // Return the minimum of the three distances
        return std::min({distAB, distBC, distCA});
    }
}
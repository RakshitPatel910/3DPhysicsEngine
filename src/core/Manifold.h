#pragma once

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

                if( dist > maxPenetration ){
                    maxPenetration = dist;
                    furthest1 = &contact;
                }
            }

            // find furthest from the line formed by the first two ===> [ 3rd point of manifold ]
            ContactData* furthest2 = nullptr;
            float maxDist2 = -std::numeric_limits<float>::max();


        }
    }

private:
    float DistanceFromLineSegment(const Vector3& point, const Vector3& A, const Vector3& B) {
        const Vector3 AB = B - A;
        const Vector3 Ap = point - A;

        
    }
}
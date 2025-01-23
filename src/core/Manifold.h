#pragma once

#include <vector>

#include "Collider.h"
#include "EPA.h"

class Manifold
{
public:
    Collider* colliderA;
    Collider* colliderB;
    
    std::vector<ContactData> persistentContact[4];
    size1_t contactCount;

    Manifold(Collider* colliderA, Collider* colliderB) : colliderA(colliderA), colliderB(colliderB) {}

    void addContactPoint(const ContactData& newPoint){
        if( contactCount == 0 ) persistentContact[0] = newPoint;

        else if( contactCount == 1 ) persistentContact[1] = newPoint;

        else if( contactCount == 2 ) {
            
        }
    }
}
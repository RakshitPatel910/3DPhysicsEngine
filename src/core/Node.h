#pragma once

#include "Vector3.h"
#include "AABB.h"

class Node
{
// private:
public:
    Node* parent;
    Node* child[2];

    AABB aabb;
    AABB* data;

    bool childCrossed;

    Node() : parent(nullptr), data(nullptr) {
        child[0] = child[1] = nullptr;
    }

    bool isLeaf() const {
        return child[0] == nullptr && child[1] == nullptr;
    }

    void setBranchNode(Node* n0, Node* n1){
        n0->parent = this;
        n1->parent = this;

        child[0] = n0;
        child[1] = n1;
    }

    void setLeafNode(AABB* d){
        this->data = d;
        d->nodeData = this;

        child[0] = nullptr;
        child[1] = nullptr;
    }

    //update AABB by margin
    void updateAABB(float margin){
        if(isLeaf()){ // making fat aabb
            const Vector3 marginV = Vector3(margin, margin, margin);

            aabb.minExt = data->minExt - marginV;
            aabb.maxExt = data->maxExt + marginV;
        }
        else { // making union aabb of both child
            aabb = AABB::merge(child[0]->aabb, child[1]->aabb);
        }
    }

    Node* getSiblingNode() const {
        return this == parent->child[0] ? parent->child[1] : parent->child[0];
    }
};


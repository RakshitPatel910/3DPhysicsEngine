#pragma once

#include<iostream>

#include "Vector3.h"
#include "AABB.h"

class AABB;

class Node {
public:
    Node* parent;
    Node* child[2];

    AABB aabb;
    AABB* data;
    
    bool childCrossed;

    Node();

    bool isLeaf() const;
    void setBranchNode(Node* n0, Node* n1);
    void setLeafNode(AABB* d);
    void updateAABB(float margin); //update AABB by margin
    Node* getSiblingNode() const;
};

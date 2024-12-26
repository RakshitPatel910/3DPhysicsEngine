#include "Node.h"
#include "AABB.h"

Node::Node() : parent(nullptr), data(nullptr), childCrossed(false) {
    child[0] = child[1] = nullptr;
}

bool Node::isLeaf() const {
    return child[0] == nullptr && child[1] == nullptr;
}

void Node::setBranchNode(Node* n0, Node* n1) {
    n0->parent = this;
    n1->parent = this;

    child[0] = n0;
    child[1] = n1;
}

void Node::setLeafNode(AABB* d) {
    this->data = d;
    d->nodeData = this;

    child[0] = nullptr;
    child[1] = nullptr;
}

void Node::updateAABB(float margin) {
    if (isLeaf()) { // making fat AABB
        const Vector3 marginV = Vector3(margin, margin, margin);
        aabb.minExt = data->minExt - marginV;
        aabb.maxExt = data->maxExt + marginV;
    } else { // making union AABB of both children
        aabb = AABB::merge(child[0]->aabb, child[1]->aabb);
    }
}

Node* Node::getSiblingNode() const {
    return this == parent->child[0] ? parent->child[1] : parent->child[0];
}

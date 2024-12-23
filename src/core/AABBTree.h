#pragma once

#include<vector>

#include "Broadphase.h"
#include "Node.h"

typedef std::vector<Node *> NodeList;

class AABBTree : public Broadphase
{
private:
    Node* m_root;
    float m_margin;
    ColliderPairList m_collidingPairs;
    NodeList m_invalidNodes; // list of nodes which moved outside of fat aabb

    void updateNodeHelper(Node* node, NodeList& invalidNodes);
    void insertNode(Node* node, Node** parent);
    void removeNode(Node* node);
    void computeCollidingPairsHelper(Node* n0, Node* n1);
    void clearChildCrossHelper(Node* node);
    void crossChild(Node* node);

public:
    AABBTree() : m_root(nullptr), m_margin(0.3f) {}

    virtual void Add(AABB* aabb);
    virtual void Remove(AABB* aabb);
    virtual void Update();
    virtual ColliderPairList& ComputeCollidingPairs();
    virtual void Query(const AABB* aabb, const ColliderList& output) const;

    // Pick and Ray will be done later

}
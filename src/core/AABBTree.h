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

void AABBTree::insertNode(Node* node, Node** parent){
    Node* p = *parent;

    if(p->isLeaf()){ // if parent is leaf node ==> create a new parent with merge of prev parent and node
        Node* newP = Node();

        newP->parent = p->parent;
        newP->setBranchNode(node, p);
        *parent = newP; //replace prev parent node with newP
    }
    else{ // parent is not leaf node ==> insert node on side which has less volume increase
        const AABB* aabb_n0 = p->child[0]->aabb;
        const AABB* aabb_n1 = p->child[1]->aabb;

        const float vol_inc_n0 = AABB::merge(aabb_n0, node->aabb).getVolume() - aabb_n0->getVolume(); // vol inc when node merged with n0
        const float vol_inc_n1 = AABB::merge(aabb_n1, node->aabb).getVolume() - aabb_n1->getVolume(); // vol inc when node merged with n1

        if( vol_inc_n0 < vol_inc_n1 ){
            insertNode(node, &p->child[0]);
        }
        else{
            insertNode(node, &p->child[1]);
        }
    }

    (*parent)->updateAABB(m_margin);
}

void AABBTree::Add(AABB* aabb){
    if(m_root){ // AABBTree exists
        Node* node = new Node();

        node->setLeafNode(aabb);
        node->updateAABB(m_margin);

        insertNode(node, &m_root);
    }
}
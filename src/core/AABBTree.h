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

    void insertNode(Node* node, Node** parent);
    void removeNode(Node* node);
    void getInvalidNodes(Node* node, NodeList& invalidNodes);
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

void AABBTree::removeNode(Node* node){
    Node* p = node->parent;

    if(p){ // node is not a root node
        Node* sib = node->getSiblingNode();

        if(p->parent){ // p is not a root node
            sib->parent = p->parent;
            
            if(p == p->parent->child[0]){
                p->parent->child[0] = sib;
            }
            else{
                p->parent->child[1] = sib;
            }
        }
        else{ // p is a root node
            m_root = sib;
            sib->parent = nullptr;
        }

        delete node;
        delete parent;
    }
    else{ // node is a root node
        m_root = nullptr;

        delete node;
    }
}

void AABBTree::Remove(AABB* aabb){
    Node* node = aabb->nodeData;

    aabb->nodeData = nullptr;
    node->data = nullptr;

    removeNode(node);
}

void AABBTree::getInvalidNodes(Node* node, NodeList& invalidNodes){
    if(node->isLeaf()){
        if(!node->aabb.contains(*(node->data))){
            invalidNodes.push_back(node);
        }
    }
    else{
        getInvalidNodes(node->child[0], invalidNodes);
        getInvalidNodes(node->child[1], invalidNodes);
    }
}

void AABBTree::Update(){
    if(m_root){
        if(m_root->isLeaf()){
            m_root->updateAABB(m_margin);
        }
        else{
            m_invalidNodes.clear();
            getInvalidNodes(m_root, m_invalidNodes);

            for(auto node : m_invalidNodes){
                Node* p = node->parent;
                Node* sib = node->getSiblingNode();
                Node** pPtr = &m_root; // ptr to pointer to p

                if(p->parent){ // p is not a root node
                    sib->parent = p->parent;

                    if(p == p->parent->child[0]){
                        pPtr = &p->parent->child[0]; // change p to left child
                    }
                    else{
                        pPtr = &p->parent->child[1]; // change p to right child
                    }
                }
                else{ // p is a root node
                    sib->parent = nullptr;
                }

                *pPtr = sib;

                delete p;

                node->updateAABB(m_margin);
                insertNode(node, &m_root);
            }

            m_invalidNodes.clear();
        }
    }
}
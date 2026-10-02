#include <iostream>
#include <cassert>
#include "CCollisionOBBBox.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"

using namespace chai3d;

int main()
{
    std::cout << "--- BAT DAU KIEM TRA GIAI DOAN 1 ---\n";

    // 1. Kiem tra cCollisionOBBBox: gan gia tri tam va do dai[cite: 1]
    cCollisionOBBBox box;
    box.m_center.set(2.0, 3.0, 4.0);
    box.m_extent.set(0.5, 1.0, 1.5);
    
    assert(box.m_center.x() == 2.0);
    assert(box.m_extent.z() == 1.5);
    assert(box.u[0].x() == 1.0 && box.u[0].y() == 0.0);
    std::cout << "[PASS] cCollisionOBBBox khoi tao va luu tru chinh xac.\n";

    // 2. Kiem tra tao cay thu cong: 1 Node Internal va 2 Node Leaf[cite: 1]
    cCollisionOBBInternal* root = new cCollisionOBBInternal();
    root->m_bbox = box;
    assert(root->m_nodeType == C_COLLISION_OBB_NODE_INTERNAL);

    cCollisionOBBLeaf* leafLeft = new cCollisionOBBLeaf();
    leafLeft->m_bbox.m_center.set(-1.0, 0.0, 0.0);
    assert(leafLeft->m_nodeType == C_COLLISION_OBB_NODE_LEAF);

    cCollisionOBBLeaf* leafRight = new cCollisionOBBLeaf();
    leafRight->m_bbox.m_center.set(1.0, 0.0, 0.0);
    assert(leafRight->m_nodeType == C_COLLISION_OBB_NODE_LEAF);

    // Gan vao nhanh con
    root->m_leftSubTree = leafLeft;
    root->m_rightSubTree = leafRight;
    std::cout << "[PASS] Cay phan cap duoc lien ket thanh cong.\n";

    // 3. Kiem tra thu hoi bo nho: Delete root phai tu dong giai phong 2 leaf
    delete root;
    root = nullptr;
    std::cout << "[PASS] Thu hoi bo nho an toan, khong co loi crash.\n";

    std::cout << "--- HOAN THANH GIAI DOAN 1 THANH CONG! ---\n";
    return 0;
}
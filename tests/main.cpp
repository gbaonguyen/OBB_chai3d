#include <iostream>
#include <vector>
#include <cassert>
#include "CCollisionOBBBox.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"
#include "CCollisionOBB.h"

void runMathTests();
void runTraversalTests();

using namespace chai3d;

void runTreeTests()
{
    std::cout << "\n--- BAT DAU KIEM TRA GIAI DOAN 3 (TOP-DOWN TREE) ---\n";

    cVector3d p0( 1.0,  1.0,  1.0);
    cVector3d p1(-1.0, -1.0,  1.0);
    cVector3d p2(-1.0,  1.0, -1.0);
    cVector3d p3( 1.0, -1.0, -1.0);

    std::vector<cTriangle*> triangles;
    triangles.push_back(new cTriangle(p0, p1, p2, 0));
    triangles.push_back(new cTriangle(p0, p1, p3, 1));
    triangles.push_back(new cTriangle(p0, p2, p3, 2));
    triangles.push_back(new cTriangle(p1, p2, p3, 3));

    cCollisionOBB obbTree;
    obbTree.initialize(triangles);

    assert(obbTree.m_root != nullptr && "Cay OBB chua duoc khoi tao root!");
    std::cout << "[PASS] Root OBB duoc khoi tao thanh cong.\n";

    int leafCount = obbTree.getLeafCount(obbTree.m_root);
    assert(leafCount == 4 && "So luong node la khong khop voi so tam giac!");
    std::cout << "[PASS] So luong leaf node chinh xac: " << leafCount << " / 4 tam giac.\n";

    int depth = obbTree.getTreeDepth(obbTree.m_root);
    assert(depth >= 2 && depth <= 4 && "Do sau cay khong hop ly!");
    std::cout << "[PASS] Do sau cay (Tree Depth) hop ly: " << depth << " levels.\n";

    assert(obbTree.m_root->m_bbox.m_extent.x() > 0.0);
    assert(obbTree.m_root->m_bbox.m_extent.y() > 0.0);
    assert(obbTree.m_root->m_bbox.m_extent.z() > 0.0);
    std::cout << "[PASS] Root Bounding Box co extents hop le.\n";

    for (auto tri : triangles) delete tri;

    std::cout << "--- HOAN THANH GIAI DOAN 3 THANH CONG! ---\n\n";
}

int main()
{
    std::cout << "=== RUNNING OBB TESTS ===\n";

    // Giai doan 1
    cCollisionOBBBox box;
    box.m_center.set(2.0, 3.0, 4.0);
    box.m_extent.set(0.5, 1.0, 1.5);
    assert(box.m_center.x() == 2.0);
    std::cout << "[PASS] Giai doan 1 (Data Structures) OK.\n";

    // Giai doan 2
    runMathTests();

    // Giai doan 3
    runTreeTests();

    // Giai doan 4
    runTraversalTests();

    std::cout << "=== ALL TESTS PASSED SUCCESSFULLY! ===\n";
    return 0;
}
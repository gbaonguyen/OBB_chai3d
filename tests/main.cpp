#include <iostream>
#include <cassert>
#include "CCollisionOBBBox.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"

// Khai bao nguyen mau ham test tu test_math.cpp
void runMathTests();

using namespace chai3d;

int main()
{
    std::cout << "=== RUNNING OBB TESTS ===\n";

    // Giai doan 1 tests
    cCollisionOBBBox box;
    box.m_center.set(2.0, 3.0, 4.0);
    box.m_extent.set(0.5, 1.0, 1.5);
    assert(box.m_center.x() == 2.0);

    cCollisionOBBInternal* root = new cCollisionOBBInternal();
    root->m_bbox = box;
    cCollisionOBBLeaf* leafLeft = new cCollisionOBBLeaf();
    cCollisionOBBLeaf* leafRight = new cCollisionOBBLeaf();
    root->m_leftSubTree = leafLeft;
    root->m_rightSubTree = leafRight;
    delete root;
    std::cout << "[PASS] Giai doan 1 (Data Structures) OK.\n";

    // Giai doan 2 tests
    runMathTests();

    std::cout << "=== ALL TESTS PASSED SUCCESSFULLY! ===\n";
    return 0;
}
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "CCollisionOBBBox.h"
#include "CCollisionOBBLeaf.h"
#include "CCollisionOBB.h"

using namespace chai3d;

void runTraversalTests()
{
    std::cout << "\n--- BAT DAU KIEM TRA GIAI DOAN 4 (COLLISION TRAVERSAL) ---\n";

    // 1. Coarse Test: Segment vs OBBBox
    cCollisionOBBBox box;
    box.m_center.set(0.0, 0.0, 0.0);
    box.m_extent.set(1.0, 1.0, 1.0);

    cVector3d ray1A(-2.0, 0.0, 0.0);
    cVector3d ray1B( 2.0, 0.0, 0.0);
    assert(box.intersectSegment(ray1A, ray1B) == true && "Tia xuyen tam phai giao OBB!");
    std::cout << "[PASS] Coarse Test: Tia xuyen tam trung hop.\n";

    cVector3d ray2A(-2.0, 3.0, 0.0);
    cVector3d ray2B( 2.0, 3.0, 0.0);
    assert(box.intersectSegment(ray2A, ray2B) == false && "Tia lech ngoai phai truot OBB!");
    std::cout << "[PASS] Coarse Test: Tia lech ngoai truot hop hop le.\n";

    // 2. Fine Test: Segment vs Triangle (Moller-Trumbore)
    cTriangle tri(cVector3d(-1.0, -1.0, 0.0),
                  cVector3d( 1.0, -1.0, 0.0),
                  cVector3d( 0.0,  1.0, 0.0), 101);

    cVector3d hitPoint, hitNormal;
    double tHit = 0.0;
    cVector3d rayZ_A(0.0, 0.0, -2.0);
    cVector3d rayZ_B(0.0, 0.0,  2.0);
    bool hitTri = tri.computeCollision(rayZ_A, rayZ_B, hitPoint, hitNormal, tHit);
    assert(hitTri == true && "Tia phai xuyen qua tam giac!");
    assert(std::fabs(hitPoint.z()) < 1e-4 && "Diem giao cat phai o mat phang z = 0!");
    std::cout << "[PASS] Fine Test: Xuyen tam giac tai toa do (" 
              << hitPoint.x() << ", " << hitPoint.y() << ", " << hitPoint.z() << ").\n";

    // 3. Toan bo cay OBB qua computeCollision()
    std::vector<cTriangle*> mesh;
    mesh.push_back(new cTriangle(cVector3d(-2.0, -2.0, 0.0), cVector3d(0.0, -2.0, 0.0), cVector3d(-1.0, 0.0, 0.0), 1));
    mesh.push_back(new cTriangle(cVector3d( 0.0,  0.0, 0.0), cVector3d(2.0,  0.0, 0.0), cVector3d( 1.0, 2.0, 0.0), 2));

    cCollisionOBB obbTree;
    obbTree.initialize(mesh);

    cCollisionRecorder recorder;
    cCollisionSettings settings;

    cVector3d testRayA(1.0, 0.5, -3.0);
    cVector3d testRayB(1.0, 0.5,  3.0);

    bool treeHit = obbTree.computeCollision(nullptr, testRayA, testRayB, recorder, settings);
    assert(treeHit == true && "Cay OBB phai phat hien duoc va cham!");
    assert(!recorder.m_collisions.empty() && "Recorder phai chua su kien va cham!");
    assert(recorder.m_collisions[0].m_index == 2 && "Va cham phai xay ra tren tam giac 2!");
    std::cout << "[PASS] Cay OBB DFS phat hien va cham dung tam giac muc tieu (Index: "
              << recorder.m_collisions[0].m_index << ").\n";

    for (auto t : mesh) delete t;

    std::cout << "--- HOAN THANH GIAI DOAN 4 THANH CONG! ---\n\n";
}
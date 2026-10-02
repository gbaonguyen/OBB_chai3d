#ifndef CCOLLISION_OBB_H
#define CCOLLISION_OBB_H

#include "collisions/CGenericCollision.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"
#include <vector>

namespace chai3d {

class cCollisionOBB : public cGenericCollision
{
public:
    // Con tro toi Node goc cua cay OBB[cite: 3]
    cCollisionOBBNode* m_root;

    // Danh sach con tro chua cac tam giac cua doi tuong 3D[cite: 3]
    std::vector<cTriangle*> m_triangles;

    // Co bat/tat Local Search[cite: 3]
    bool m_useNeighbors;

    cCollisionOBB();
    virtual ~cCollisionOBB();

    // Ham khoi tao va dung cay OBB tu danh sach tam giac[cite: 3]
    void initialize(const std::vector<cTriangle*>& a_triangles);
    virtual void initialize(const double a_radius = 0.0);

    // Ham kiem tra va cham tong the (Giai doan 4)[cite: 1, 3]
    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override;

    // Ham render khung day OBB (Giai doan 5)[cite: 1, 3]
    virtual void render(cRenderOptions& a_options) override;

    // Ham tien ich phuc vu kiem thu cau truc cay
    int getLeafCount(cCollisionOBBNode* a_node) const;
    int getTreeDepth(cCollisionOBBNode* a_node) const;

private:
    // Ham de quy cot loi dung cay nhan danh sach tam giac con
    cCollisionOBBNode* buildTree(std::vector<cTriangle*>& a_triangles, int a_depth);
};

} // namespace chai3d

#endif // CCOLLISION_OBB_H
#ifndef CCOLLISION_OBB_LEAF_H
#define CCOLLISION_OBB_LEAF_H

#include "CCollisionOBBNode.h"

namespace chai3d {

// Khai báo trước để trình biên dịch biết kiểu cTriangle
class cTriangle;

class cCollisionOBBLeaf : public cCollisionOBBNode
{
public:
    // Con trỏ tới tam giác hình học thực tế[cite: 1]
    cTriangle* m_triangle;

    cCollisionOBBLeaf()
    {
        m_nodeType = C_COLLISION_OBB_NODE_LEAF;
        m_triangle = nullptr;
    }

    cCollisionOBBLeaf(cTriangle* a_triangle)
    {
        m_nodeType = C_COLLISION_OBB_NODE_LEAF;
        m_triangle = a_triangle;
    }

    virtual ~cCollisionOBBLeaf() {}

    // Tạm thời trả về false, sẽ xử lý logic Fine test ở Giai đoạn 4[cite: 1]
    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override
    {
        return false;
    }

    // Tạm thời để trống, sẽ cài đặt ở Giai đoạn 5[cite: 1]
    virtual void render(cRenderOptions& a_options) override {}
};

} // namespace chai3d

#endif // CCOLLISION_OBB_LEAF_H
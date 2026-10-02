#ifndef CCOLLISION_OBB_INTERNAL_H
#define CCOLLISION_OBB_INTERNAL_H

#include "CCollisionOBBNode.h"

namespace chai3d{
class cCollisionOBBInternal : public cCollisionOBBNode {
public:
    cCollisionOBBNode* m_leftSubTree;
    cCollisionOBBNode* m_rightSubTree;

    cCollisionOBBInternal(){
        m_nodeType = C_COLLISION_OBB_NODE_INTERNAL;
        m_leftSubTree = nullptr;
        m_rightSubTree = nullptr;
    }

    virtual ~cCollisionOBBInternal(){
        if (m_leftSubTree != nullptr){
            delete m_leftSubTree;
            m_leftSubTree = nullptr;
        }
        if (m_rightSubTree != nullptr){
            delete m_rightSubTree;
            m_rightSubTree = nullptr;
        }

    }

    

    // Tạm thời trả về false, sẽ xử lý logic ở Giai đoạn 4[cite: 1]
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

}

#endif 
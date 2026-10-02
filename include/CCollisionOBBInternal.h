#ifndef CCOLLISION_OBB_INTERNAL_H
#define CCOLLISION_OBB_INTERNAL_H

#include "CCollisionOBBNode.h"

namespace chai3d {

class cCollisionOBBInternal : public cCollisionOBBNode
{
public:
    cCollisionOBBNode* m_leftSubTree;
    cCollisionOBBNode* m_rightSubTree;

    cCollisionOBBInternal()
    {
        m_nodeType = C_COLLISION_OBB_NODE_INTERNAL;
        m_leftSubTree = nullptr;
        m_rightSubTree = nullptr;
    }

    virtual ~cCollisionOBBInternal()
    {
        if (m_leftSubTree != nullptr)
        {
            delete m_leftSubTree;
            m_leftSubTree = nullptr;
        }
        if (m_rightSubTree != nullptr)
        {
            delete m_rightSubTree;
            m_rightSubTree = nullptr;
        }
    }

    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override
    {
        // 1. Coarse test: Doan thang vs OBB cua node trung gian
        if (!m_bbox.intersectSegment(a_segmentPointA, a_segmentPointB))
            return false;

        // 2. DFS: De quy sang 2 nhanh con
        bool hitLeft = false;
        bool hitRight = false;

        if (m_leftSubTree != nullptr)
        {
            hitLeft = m_leftSubTree->computeCollision(a_object, a_segmentPointA, a_segmentPointB, a_recorder, a_settings);
        }

        if (m_rightSubTree != nullptr)
        {
            hitRight = m_rightSubTree->computeCollision(a_object, a_segmentPointA, a_segmentPointB, a_recorder, a_settings);
        }

        return (hitLeft || hitRight);
    }

    virtual void render(cRenderOptions& a_options) override {}
};

} // namespace chai3d

#endif // CCOLLISION_OBB_INTERNAL_H
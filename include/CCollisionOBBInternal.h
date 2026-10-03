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
        if (!m_bbox.intersectSegment(a_segmentPointA, a_segmentPointB))
            return false;

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

    virtual void render(cRenderOptions& a_options) override
    {
        render(a_options, 0, -1);
    }

    virtual void render(cRenderOptions& a_options, int a_currentDepth, int a_targetDepth) override
    {
        // Neu muc tieu la -1 (ve tat ca) hoac dung tang do sau can render
        if (a_targetDepth == -1 || a_currentDepth == a_targetDepth)
        {
            m_bbox.render();
        }

        // Neu chua dat den do sau toi da thi tiep tuc de quy xuong cac cay con
        if (a_targetDepth == -1 || a_currentDepth < a_targetDepth)
        {
            if (m_leftSubTree != nullptr)
                m_leftSubTree->render(a_options, a_currentDepth + 1, a_targetDepth);

            if (m_rightSubTree != nullptr)
                m_rightSubTree->render(a_options, a_currentDepth + 1, a_targetDepth);
        }
    }
};

} // namespace chai3d

#endif // CCOLLISION_OBB_INTERNAL_H
#include "CCollisionOBBInternal.h"

namespace chai3d {

cCollisionOBBInternal::cCollisionOBBInternal()
{
    m_nodeType = C_COLLISION_OBB_NODE_INTERNAL;
    m_leftSubTree = nullptr;
    m_rightSubTree = nullptr;
}

cCollisionOBBInternal::~cCollisionOBBInternal()
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

bool cCollisionOBBInternal::computeCollision(cGenericObject* a_object,
                                             cVector3d& a_segmentPointA,
                                             cVector3d& a_segmentPointB,
                                             cCollisionRecorder& a_recorder,
                                             cCollisionSettings& a_settings)
{
    // Reject the whole subtree before recursively testing either child.
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

void cCollisionOBBInternal::render(cRenderOptions& a_options)
{
    render(a_options, 0, -1);
}

void cCollisionOBBInternal::render(cRenderOptions& a_options, int a_currentDepth, int a_targetDepth)
{
    // if targetDepth is -1, render all nodes; otherwise, only render nodes at the specified depth
    if (a_targetDepth == -1 || a_currentDepth == a_targetDepth)
    {
        m_bbox.render();
    }

    // If the target depth is -1 or the current depth is less than the target depth, continue recursively down the tree
    if (a_targetDepth == -1 || a_currentDepth < a_targetDepth)
    {
        if (m_leftSubTree != nullptr)
            m_leftSubTree->render(a_options, a_currentDepth + 1, a_targetDepth);

        if (m_rightSubTree != nullptr)
            m_rightSubTree->render(a_options, a_currentDepth + 1, a_targetDepth);
    }
}

} // namespace chai3d
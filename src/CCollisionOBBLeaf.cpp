#include "CCollisionOBBLeaf.h"

namespace chai3d {

cCollisionOBBLeaf::cCollisionOBBLeaf()
{
    m_nodeType = C_COLLISION_OBB_NODE_LEAF;
    m_triangle = nullptr;
}

cCollisionOBBLeaf::cCollisionOBBLeaf(cTriangle* a_triangle)
{
    m_nodeType = C_COLLISION_OBB_NODE_LEAF;
    m_triangle = a_triangle;
}

cCollisionOBBLeaf::~cCollisionOBBLeaf() {}

bool cCollisionOBBLeaf::computeCollision(cGenericObject* a_object,
                                         cVector3d& a_segmentPointA,
                                         cVector3d& a_segmentPointB,
                                         cCollisionRecorder& a_recorder,
                                         cCollisionSettings& a_settings)
{
    // The box test is a cheap rejection before the exact triangle test.
    if (!m_bbox.intersectSegment(a_segmentPointA, a_segmentPointB))
        return false;

    if (m_triangle == nullptr) return false;

    cVector3d hitPoint, hitNormal;
    double tHit = 0.0;
    if (m_triangle->computeCollision(a_segmentPointA, a_segmentPointB, hitPoint, hitNormal, tHit))
    {
        // Chai3D consumes collision events from the recorder rather than a
        // single return value, so retain the complete hit information here.
        cCollisionEvent event;
        event.m_object = a_object;
        event.m_index = m_triangle->m_index;
        event.m_localPos = hitPoint;
        event.m_globalPos = hitPoint;
        event.m_localNormal = hitNormal;
        event.m_globalNormal = hitNormal;
        event.m_squareDistance = (hitPoint - a_segmentPointA).lengthsq();

        a_recorder.m_collisions.push_back(event);
        return true;
    }

    return false;
}

void cCollisionOBBLeaf::render(cRenderOptions& a_options)
{
    render(a_options, 0, -1);
}

void cCollisionOBBLeaf::render(cRenderOptions& a_options, int a_currentDepth, int a_targetDepth)
{
    if (a_targetDepth == -1 || a_currentDepth == a_targetDepth)
    {
        m_bbox.render();
    }
}

} // namespace chai3d
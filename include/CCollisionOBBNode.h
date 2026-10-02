#ifndef CCOLLISION_OBB_NODE_H
#define CCOLLISION_OBB_NODE_H

#include "CCollisionOBBBox.h"
#include "collisions/CGenericCollision.h"

namespace chai3d {

const int C_COLLISION_OBB_NODE_INTERNAL = 0;
const int C_COLLISION_OBB_NODE_LEAF     = 1;

class cCollisionOBBNode
{
public:
    cCollisionOBBBox m_bbox;
    int m_nodeType;

    cCollisionOBBNode()
    {
        m_nodeType = C_COLLISION_OBB_NODE_INTERNAL;
    }

    virtual ~cCollisionOBBNode() {}

    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) = 0;

    virtual void render(cRenderOptions& a_options) = 0;
};

} // namespace chai3d

#endif // CCOLLISION_OBB_NODE_H
#ifndef CCOLLISION_OBB_LEAF_H
#define CCOLLISION_OBB_LEAF_H

#include "CCollisionOBBNode.h"
#include "CTriangle.h"

namespace chai3d {

// Leaf node containing the exact triangle test for one mesh triangle.
class cCollisionOBBLeaf : public cCollisionOBBNode
{
public:
    cTriangle* m_triangle;

    cCollisionOBBLeaf();
    explicit cCollisionOBBLeaf(cTriangle* a_triangle);
    virtual ~cCollisionOBBLeaf();

    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings) override;

    virtual void render(cRenderOptions& a_options) override;
    virtual void render(cRenderOptions& a_options, int a_currentDepth, int a_targetDepth) override;
};

} // namespace chai3d

#endif // CCOLLISION_OBB_LEAF_H
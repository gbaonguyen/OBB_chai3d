#ifndef CCOLLISION_OBB_INTERNAL_H
#define CCOLLISION_OBB_INTERNAL_H

#include "CCollisionOBBNode.h"

namespace chai3d {

// Internal hierarchy node that delegates tests to both child subtrees.
class cCollisionOBBInternal : public cCollisionOBBNode
{
public:
    cCollisionOBBNode* m_leftSubTree;
    cCollisionOBBNode* m_rightSubTree;

    cCollisionOBBInternal();
    virtual ~cCollisionOBBInternal();

    virtual bool computeCollision(cGenericObject* a_object,
                                  cVector3d& a_segmentPointA,
                                  cVector3d& a_segmentPointB,
                                  cCollisionRecorder& a_recorder,
                                  cCollisionSettings& a_settings) override;

    virtual void render(cRenderOptions& a_options) override;
    virtual void render(cRenderOptions& a_options, int a_currentDepth, int a_targetDepth) override;
};

} // namespace chai3d

#endif // CCOLLISION_OBB_INTERNAL_H
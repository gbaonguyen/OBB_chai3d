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
    cCollisionOBBNode* m_root;
    std::vector<cTriangle*> m_triangles;
    bool m_useNeighbors;

    // Cac bien dieu khien truc quan hoa Giai doan 5
    bool m_showBoundingBoxes;
    int m_displayDepth; // -1: tat ca tang; >= 0: chi hien thi tang cu the

    cCollisionOBB();
    virtual ~cCollisionOBB();

    void initialize(const std::vector<cTriangle*>& a_triangles);
    virtual void initialize(const double a_radius = 0.0);

    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override;

    virtual void render(cRenderOptions& a_options) override;

    void setDisplayDepth(int a_depth) { m_displayDepth = a_depth; }
    int getDisplayDepth() const { return m_displayDepth; }

    int getLeafCount(cCollisionOBBNode* a_node) const;
    int getTreeDepth(cCollisionOBBNode* a_node) const;

private:
    cCollisionOBBNode* buildTree(std::vector<cTriangle*>& a_triangles, int a_depth);
};

} // namespace chai3d

#endif // CCOLLISION_OBB_H
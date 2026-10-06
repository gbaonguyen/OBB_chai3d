#ifndef CCOLLISION_OBB_H
#define CCOLLISION_OBB_H

#include "collisions/CGenericCollision.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"
#include <vector>

namespace chai3d {

// Collision detector backed by a hierarchy of oriented bounding boxes.
class cCollisionOBB : public cGenericCollision
{
public:
    // The root owns the complete hierarchy; child nodes are released by it.
    cCollisionOBBNode* m_root;
    std::vector<cTriangle*> m_triangles;
    bool m_useNeighbors;

    // Pointer to the last triangle that was collided with. This is used for local search.
    cTriangle* m_lastCollidedTriangle;

    // Variables for rendering the bounding boxes of the hierarchy.
    bool m_showBoundingBoxes;
    int m_displayDepth;

    cCollisionOBB();
    virtual ~cCollisionOBB();

    // Stores the mesh triangles and rebuilds the hierarchy.
    void initialize(const std::vector<cTriangle*>& a_triangles);
    virtual void initialize(const double a_radius = 0.0);

    // Builds the neighbor relationships between leaf nodes for local search.
    void buildNeighbors();

    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override;

    virtual void render(cRenderOptions& a_options) override;

    // A depth of -1 renders every box in the hierarchy.
    void setDisplayDepth(int a_depth) { m_displayDepth = a_depth; }
    int getDisplayDepth() const { return m_displayDepth; }

    int getLeafCount(cCollisionOBBNode* a_node) const;
    int getTreeDepth(cCollisionOBBNode* a_node) const;

private:
    cCollisionOBBNode* buildTree(std::vector<cTriangle*>& a_triangles, int a_depth);
};

} // namespace chai3d

#endif // CCOLLISION_OBB_H
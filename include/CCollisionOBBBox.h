#ifndef CCOLLISION_OBB_BOX_H
#define CCOLLISION_OBB_BOX_H

#include "math/CVector3d.h"

namespace chai3d {

// An oriented box represented by its center, half-extents, and local axes.
class cCollisionOBBBox
{
public:
    cVector3d m_center;
    cVector3d m_extent;
    cVector3d u[3];

    cCollisionOBBBox();
    virtual ~cCollisionOBBBox();

    // Coarse test using the slab intersection method in box-local coordinates.
    bool intersectSegment(const cVector3d& a_pA, const cVector3d& a_pB) const;

    // Draws the twelve box edges using the active OpenGL context.
    void render() const;
};

} // namespace chai3d

#endif // CCOLLISION_OBB_BOX_H
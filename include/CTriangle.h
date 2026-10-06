#ifndef CTRIANGLE_H
#define CTRIANGLE_H

#include <vector>
#include "math/CVector3d.h"

namespace chai3d {

// Lightweight triangle record used by the OBB hierarchy and exact tests.
class cTriangle
{
public:
    cVector3d m_v0;
    cVector3d m_v1;
    cVector3d m_v2;
    int m_index;

    // List of neighboring triangles that share an edge with this triangle.
    std::vector<cTriangle*> m_neighbors;

    cTriangle();
    cTriangle(const cVector3d& a_v0, const cVector3d& a_v1, const cVector3d& a_v2, int a_index = -1);

    cVector3d getVertex0() const;
    cVector3d getVertex1() const;
    cVector3d getVertex2() const;

    cVector3d computeCentroid() const;

    // Fine test: segment versus triangle using Moller-Trumbore barycentrics.
    bool computeCollision(const cVector3d& a_pA,
                          const cVector3d& a_pB,
                          cVector3d& a_hitPoint,
                          cVector3d& a_hitNormal,
                          double& a_tHit) const;
};

} // namespace chai3d

#endif // CTRIANGLE_H
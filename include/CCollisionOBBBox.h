#ifndef CCOLLISION_OBB_BOX_H
#define CCOLLISION_OBB_BOX_H

#include "math/CVector3d.h"
#include <cmath>
#include <algorithm>

namespace chai3d {

class cCollisionOBBBox
{
public:
    cVector3d m_center;
    cVector3d m_extent;
    cVector3d u[3];

    cCollisionOBBBox()
    {
        m_center.zero();
        m_extent.zero();
        u[0].set(1.0, 0.0, 0.0);
        u[1].set(0.0, 1.0, 0.0);
        u[2].set(0.0, 0.0, 1.0);
    }

    virtual ~cCollisionOBBBox() {}

    // Coarse Test: Kiem tra giao cat doan thang AB voi OBB bang Slab Method
    bool intersectSegment(const cVector3d& a_pA, const cVector3d& a_pB) const
    {
        cVector3d diff = a_pA - m_center;
        cVector3d dir  = a_pB - a_pA;

        double aLoc[3] = {
            diff.x() * u[0].x() + diff.y() * u[0].y() + diff.z() * u[0].z(),
            diff.x() * u[1].x() + diff.y() * u[1].y() + diff.z() * u[1].z(),
            diff.x() * u[2].x() + diff.y() * u[2].y() + diff.z() * u[2].z()
        };

        double dLoc[3] = {
            dir.x() * u[0].x() + dir.y() * u[0].y() + dir.z() * u[0].z(),
            dir.x() * u[1].x() + dir.y() * u[1].y() + dir.z() * u[1].z(),
            dir.x() * u[2].x() + dir.y() * u[2].y() + dir.z() * u[2].z()
        };

        double ext[3] = { m_extent.x(), m_extent.y(), m_extent.z() };

        double tMin = 0.0;
        double tMax = 1.0;

        for (int i = 0; i < 3; ++i)
        {
            if (std::fabs(dLoc[i]) < 1e-9)
            {
                if (aLoc[i] < -ext[i] || aLoc[i] > ext[i])
                    return false;
            }
            else
            {
                double invD = 1.0 / dLoc[i];
                double t1 = (-ext[i] - aLoc[i]) * invD;
                double t2 = ( ext[i] - aLoc[i]) * invD;

                if (t1 > t2) std::swap(t1, t2);

                tMin = std::max(tMin, t1);
                tMax = std::min(tMax, t2);

                if (tMin > tMax) return false;
            }
        }

        return (tMin <= tMax && tMax >= 0.0 && tMin <= 1.0);
    }
};

} // namespace chai3d

#endif // CCOLLISION_OBB_BOX_H
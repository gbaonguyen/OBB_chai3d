#include "CCollisionOBBBox.h"
#include "graphics/COpenGLHeaders.h"
#include <cmath>
#include <algorithm>

namespace chai3d {

// Start with an axis-aligned unit frame so a default box is immediately usable.
cCollisionOBBBox::cCollisionOBBBox()
{
    m_center.zero();
    m_extent.zero();
    u[0].set(1.0, 0.0, 0.0);
    u[1].set(0.0, 1.0, 0.0);
    u[2].set(0.0, 0.0, 1.0);
}

cCollisionOBBBox::~cCollisionOBBBox() {}

bool cCollisionOBBBox::intersectSegment(const cVector3d& a_pA, const cVector3d& a_pB) const
{
    // Transform the segment into the box frame; each loop clips one slab.
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

void cCollisionOBBBox::render() const
{
    // Build the eight corners from the oriented basis and half-extents.
    cVector3d ex = u[0] * m_extent.x();
    cVector3d ey = u[1] * m_extent.y();
    cVector3d ez = u[2] * m_extent.z();

    cVector3d v[8];
    v[0] = m_center - ex - ey - ez;
    v[1] = m_center + ex - ey - ez;
    v[2] = m_center + ex + ey - ez;
    v[3] = m_center - ex + ey - ez;
    v[4] = m_center - ex - ey + ez;
    v[5] = m_center + ex - ey + ez;
    v[6] = m_center + ex + ey + ez;
    v[7] = m_center - ex + ey + ez;

    glBegin(GL_LINES);

    // under face (z-)
    glVertex3d(v[0].x(), v[0].y(), v[0].z()); glVertex3d(v[1].x(), v[1].y(), v[1].z());
    glVertex3d(v[1].x(), v[1].y(), v[1].z()); glVertex3d(v[2].x(), v[2].y(), v[2].z());
    glVertex3d(v[2].x(), v[2].y(), v[2].z()); glVertex3d(v[3].x(), v[3].y(), v[3].z());
    glVertex3d(v[3].x(), v[3].y(), v[3].z()); glVertex3d(v[0].x(), v[0].y(), v[0].z());

    // upper face (z+)
    glVertex3d(v[4].x(), v[4].y(), v[4].z()); glVertex3d(v[5].x(), v[5].y(), v[5].z());
    glVertex3d(v[5].x(), v[5].y(), v[5].z()); glVertex3d(v[6].x(), v[6].y(), v[6].z());
    glVertex3d(v[6].x(), v[6].y(), v[6].z()); glVertex3d(v[7].x(), v[7].y(), v[7].z());
    glVertex3d(v[7].x(), v[7].y(), v[7].z()); glVertex3d(v[4].x(), v[4].y(), v[4].z());

    // 4 vertical edges
    glVertex3d(v[0].x(), v[0].y(), v[0].z()); glVertex3d(v[4].x(), v[4].y(), v[4].z());
    glVertex3d(v[1].x(), v[1].y(), v[1].z()); glVertex3d(v[5].x(), v[5].y(), v[5].z());
    glVertex3d(v[2].x(), v[2].y(), v[2].z()); glVertex3d(v[6].x(), v[6].y(), v[6].z());
    glVertex3d(v[3].x(), v[3].y(), v[3].z()); glVertex3d(v[7].x(), v[7].y(), v[7].z());

    glEnd();
}

} // namespace chai3d
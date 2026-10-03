#include "CCollisionOBBBox.h"
#include "graphics/COpenGLHeaders.h"

namespace chai3d {

void cCollisionOBBBox::render() const
{
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

    // Mat day (z-)
    glVertex3d(v[0].x(), v[0].y(), v[0].z()); glVertex3d(v[1].x(), v[1].y(), v[1].z());
    glVertex3d(v[1].x(), v[1].y(), v[1].z()); glVertex3d(v[2].x(), v[2].y(), v[2].z());
    glVertex3d(v[2].x(), v[2].y(), v[2].z()); glVertex3d(v[3].x(), v[3].y(), v[3].z());
    glVertex3d(v[3].x(), v[3].y(), v[3].z()); glVertex3d(v[0].x(), v[0].y(), v[0].z());

    // Mat tren (z+)
    glVertex3d(v[4].x(), v[4].y(), v[4].z()); glVertex3d(v[5].x(), v[5].y(), v[5].z());
    glVertex3d(v[5].x(), v[5].y(), v[5].z()); glVertex3d(v[6].x(), v[6].y(), v[6].z());
    glVertex3d(v[6].x(), v[6].y(), v[6].z()); glVertex3d(v[7].x(), v[7].y(), v[7].z());
    glVertex3d(v[7].x(), v[7].y(), v[7].z()); glVertex3d(v[4].x(), v[4].y(), v[4].z());

    // 4 canh dung noi hai mat
    glVertex3d(v[0].x(), v[0].y(), v[0].z()); glVertex3d(v[4].x(), v[4].y(), v[4].z());
    glVertex3d(v[1].x(), v[1].y(), v[1].z()); glVertex3d(v[5].x(), v[5].y(), v[5].z());
    glVertex3d(v[2].x(), v[2].y(), v[2].z()); glVertex3d(v[6].x(), v[6].y(), v[6].z());
    glVertex3d(v[3].x(), v[3].y(), v[3].z()); glVertex3d(v[7].x(), v[7].y(), v[7].z());

    glEnd();
}

} // namespace chai3d
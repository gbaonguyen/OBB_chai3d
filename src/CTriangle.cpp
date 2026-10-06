#include "CTriangle.h"
#include <cmath>

namespace chai3d {

namespace {

// Keep vector operations local to this translation unit.
cVector3d crossProd(const cVector3d& a, const cVector3d& b)
{
    return cVector3d(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x()
    );
}

double dotProd(const cVector3d& a, const cVector3d& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

} // anonymous namespace

cTriangle::cTriangle() : m_index(-1)
{
    m_v0.zero();
    m_v1.zero();
    m_v2.zero();
}

cTriangle::cTriangle(const cVector3d& a_v0, const cVector3d& a_v1, const cVector3d& a_v2, int a_index)
    : m_v0(a_v0), m_v1(a_v1), m_v2(a_v2), m_index(a_index)
{
}

cVector3d cTriangle::getVertex0() const { return m_v0; }
cVector3d cTriangle::getVertex1() const { return m_v1; }
cVector3d cTriangle::getVertex2() const { return m_v2; }

cVector3d cTriangle::computeCentroid() const
{
    return (m_v0 + m_v1 + m_v2) * (1.0 / 3.0);
}

bool cTriangle::computeCollision(const cVector3d& a_pA,
                                 const cVector3d& a_pB,
                                 cVector3d& a_hitPoint,
                                 cVector3d& a_hitNormal,
                                 double& a_tHit) const
{
    // The parameter t is normalized to the finite segment [a_pA, a_pB].
    cVector3d dir = a_pB - a_pA;
    cVector3d edge1 = m_v1 - m_v0;
    cVector3d edge2 = m_v2 - m_v0;

    cVector3d pvec = crossProd(dir, edge2);
    double det = dotProd(edge1, pvec);

    // A near-zero determinant means the segment is parallel to the triangle.
    if (std::fabs(det) < 1e-9) return false;

    double invDet = 1.0 / det;
    cVector3d tvec = a_pA - m_v0;

    double u = dotProd(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    cVector3d qvec = crossProd(tvec, edge1);
    double v = dotProd(dir, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;

    double t = dotProd(edge2, qvec) * invDet;
    if (t < 0.0 || t > 1.0) return false;

    a_tHit = t;
    a_hitPoint = a_pA + (dir * t);
    a_hitNormal = crossProd(edge1, edge2);
    a_hitNormal.normalize();

    return true;
}

} // namespace chai3d
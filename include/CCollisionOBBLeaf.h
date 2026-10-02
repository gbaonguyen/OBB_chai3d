#ifndef CCOLLISION_OBB_LEAF_H
#define CCOLLISION_OBB_LEAF_H

#include "CCollisionOBBNode.h"
#include <cmath>

namespace chai3d {

class cTriangle
{
public:
    cVector3d m_v0;
    cVector3d m_v1;
    cVector3d m_v2;
    int m_index;

    cTriangle() : m_index(-1)
    {
        m_v0.zero();
        m_v1.zero();
        m_v2.zero();
    }

    cTriangle(const cVector3d& a_v0, const cVector3d& a_v1, const cVector3d& a_v2, int a_index = -1)
        : m_v0(a_v0), m_v1(a_v1), m_v2(a_v2), m_index(a_index) {}

    inline cVector3d getVertex0() const { return m_v0; }
    inline cVector3d getVertex1() const { return m_v1; }
    inline cVector3d getVertex2() const { return m_v2; }

    cVector3d computeCentroid() const
    {
        return (m_v0 + m_v1 + m_v2) * (1.0 / 3.0);
    }

    // Fine Test: Giao cat Doan thang - Tam giac (Moller-Trumbore)
    bool computeCollision(const cVector3d& a_pA,
                          const cVector3d& a_pB,
                          cVector3d& a_hitPoint,
                          cVector3d& a_hitNormal,
                          double& a_tHit) const
    {
        auto crossProd = [](const cVector3d& a, const cVector3d& b) {
            return cVector3d(
                a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x()
            );
        };

        auto dotProd = [](const cVector3d& a, const cVector3d& b) {
            return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
        };

        cVector3d dir = a_pB - a_pA;
        cVector3d edge1 = m_v1 - m_v0;
        cVector3d edge2 = m_v2 - m_v0;

        cVector3d pvec = crossProd(dir, edge2);
        double det = dotProd(edge1, pvec);

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
};

class cCollisionOBBLeaf : public cCollisionOBBNode
{
public:
    cTriangle* m_triangle;

    cCollisionOBBLeaf()
    {
        m_nodeType = C_COLLISION_OBB_NODE_LEAF;
        m_triangle = nullptr;
    }

    cCollisionOBBLeaf(cTriangle* a_triangle)
    {
        m_nodeType = C_COLLISION_OBB_NODE_LEAF;
        m_triangle = a_triangle;
    }

    virtual ~cCollisionOBBLeaf() {}

    virtual bool computeCollision(cGenericObject* a_object,
                                   cVector3d& a_segmentPointA,
                                   cVector3d& a_segmentPointB,
                                   cCollisionRecorder& a_recorder,
                                   cCollisionSettings& a_settings) override
    {
        // 1. Coarse test: Doan thang vs OBB cua node la
        if (!m_bbox.intersectSegment(a_segmentPointA, a_segmentPointB))
        {
            return false;
        }

        // 2. Fine test: Kiem tra chi tiet voi tam giac
        if (m_triangle == nullptr) return false;

        cVector3d hitPoint, hitNormal;
        double tHit = 0.0;
        if (m_triangle->computeCollision(a_segmentPointA, a_segmentPointB, hitPoint, hitNormal, tHit))
        {
            cCollisionEvent event;
            event.m_object = a_object;
            event.m_index = m_triangle->m_index;
            event.m_localPos = hitPoint;
            event.m_globalPos = hitPoint;
            event.m_localNormal = hitNormal;
            event.m_globalNormal = hitNormal;
            event.m_squareDistance = (hitPoint - a_segmentPointA).lengthsq();

            a_recorder.m_collisions.push_back(event);
            return true;
        }

        return false;
    }

    virtual void render(cRenderOptions& a_options) override {}
};

} // namespace chai3d

#endif // CCOLLISION_OBB_LEAF_H
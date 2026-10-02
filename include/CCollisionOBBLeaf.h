#ifndef CCOLLISION_OBB_LEAF_H
#define CCOLLISION_OBB_LEAF_H

#include "CCollisionOBBNode.h"

namespace chai3d {

// Cau truc bieu dien mot tam giac trong khong gian 3D
class cTriangle
{
public:
    cVector3d m_v0;
    cVector3d m_v1;
    cVector3d m_v2;
    int m_index; // Chi so goc trong Mesh

    cTriangle() : m_index(-1) 
    {
        m_v0.zero();
        m_v1.zero();
        m_v2.zero();
    }

    cTriangle(const cVector3d& a_v0, const cVector3d& a_v1, const cVector3d& a_v2, int a_index = -1)
        : m_v0(a_v0), m_v1(a_v1), m_v2(a_v2), m_index(a_index) {}

    // Tinh trong tam (Centroid) cua tam giac
    cVector3d computeCentroid() const
    {
        return (m_v0 + m_v1 + m_v2) * (1.0 / 3.0);
    }
};

class cCollisionOBBLeaf : public cCollisionOBBNode
{
public:
    // Con tro toi tam giac thuc te ma node la quan ly[cite: 3]
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
        // Se cai dat Fine test o Giai doan 4
        return false;
    }

    virtual void render(cRenderOptions& a_options) override {}
};

} // namespace chai3d

#endif // CCOLLISION_OBB_LEAF_H
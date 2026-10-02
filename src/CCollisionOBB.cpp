#include "CCollisionOBB.h"
#include "OBBMath.h"
#include <algorithm>
#include <cmath>

namespace chai3d {

cCollisionOBB::cCollisionOBB()
{
    m_root = nullptr;
    m_useNeighbors = false;
}

cCollisionOBB::~cCollisionOBB()
{
    if (m_root != nullptr)
    {
        delete m_root;
        m_root = nullptr;
    }
}

void cCollisionOBB::initialize(const double a_radius)
{
    // Interface tuong thich voi cGenericCollision[cite: 3]
    if (!m_triangles.empty())
    {
        initialize(m_triangles);
    }
}

void cCollisionOBB::initialize(const std::vector<cTriangle*>& a_triangles)
{
    // Giai phong cay cu neu co
    if (m_root != nullptr)
    {
        delete m_root;
        m_root = nullptr;
    }

    m_triangles = a_triangles;
    if (m_triangles.empty()) return;

    // Bat dau dung cay de quy tu goc (Root)[cite: 1, 4]
    std::vector<cTriangle*> triList = m_triangles;
    m_root = buildTree(triList, 0);
}

cCollisionOBBNode* cCollisionOBB::buildTree(std::vector<cTriangle*>& a_triangles, int a_depth)
{
    if (a_triangles.empty()) return nullptr;

    // 1. Thu thap toan bo cac dinh cua cac tam giac thuoc node hien tai[cite: 3]
    std::vector<cVector3d> points;
    points.reserve(a_triangles.size() * 3);
    for (const auto& tri : a_triangles)
    {
        points.push_back(tri->m_v0);
        points.push_back(tri->m_v1);
        points.push_back(tri->m_v2);
    }

    // 2. Tinh toan OBB toi uu bang PCA tu tap dinh[cite: 3]
    cCollisionOBBBox nodeBox;
    buildOBBFromPoints(points, nodeBox);

    // 3. BASE CASE: Neu so luong tam giac <= 1 thi tao Node La (Leaf)[cite: 1, 4]
    if (a_triangles.size() <= 1)
    {
        cCollisionOBBLeaf* leaf = new cCollisionOBBLeaf(a_triangles[0]);
        leaf->m_bbox = nodeBox;
        return leaf;
    }

    // 4. SPLITTING STRATEGY: Tim truc dai nhat cua OBB[cite: 4]
    int longestAxis = 0;
    double maxExtent = nodeBox.m_extent.x();
    if (nodeBox.m_extent.y() > maxExtent)
    {
        maxExtent = nodeBox.m_extent.y();
        longestAxis = 1;
    }
    if (nodeBox.m_extent.z() > maxExtent)
    {
        longestAxis = 2;
    }

    cVector3d splitAxis = nodeBox.u[longestAxis];
    cVector3d splitOrigin = nodeBox.m_center;

    // 5. Phan loai tam giac vao 2 tap con (Trai va Phai) dua vao mat phang chia[cite: 4]
    std::vector<cTriangle*> leftList;
    std::vector<cTriangle*> rightList;

    for (const auto& tri : a_triangles)
    {
        cVector3d centroid = tri->computeCentroid();
        cVector3d diff = centroid - splitOrigin;

        // Tinh hinh chieu len truc dai nhat
        double proj = diff.x() * splitAxis.x() + diff.y() * splitAxis.y() + diff.z() * splitAxis.z();

        if (proj >= 0.0)
        {
            leftList.push_back(tri);
        }
        else
        {
            rightList.push_back(tri);
        }
    }

    // 6. XU LY SUY BIEN (Degenerate Case): Neu toan bo tam giac bi don sang 1 ben
    // Ap dung Median Split: Chia doi mang de dam bao cay khong bi de quy vo han
    if (leftList.empty() || rightList.empty())
    {
        leftList.clear();
        rightList.clear();
        size_t half = a_triangles.size() / 2;
        for (size_t i = 0; i < a_triangles.size(); ++i)
        {
            if (i < half) leftList.push_back(a_triangles[i]);
            else rightList.push_back(a_triangles[i]);
        }
    }

    // 7. Khoi tao Node Trung Gian va de quy dung 2 nhanh cay con[cite: 1, 3, 4]
    cCollisionOBBInternal* internalNode = new cCollisionOBBInternal();
    internalNode->m_bbox = nodeBox;
    internalNode->m_leftSubTree = buildTree(leftList, a_depth + 1);
    internalNode->m_rightSubTree = buildTree(rightList, a_depth + 1);

    return internalNode;
}

int cCollisionOBB::getLeafCount(cCollisionOBBNode* a_node) const
{
    if (a_node == nullptr) return 0;
    if (a_node->m_nodeType == C_COLLISION_OBB_NODE_LEAF) return 1;

    cCollisionOBBInternal* internalNode = dynamic_cast<cCollisionOBBInternal*>(a_node);
    if (internalNode != nullptr)
    {
        return getLeafCount(internalNode->m_leftSubTree) + getLeafCount(internalNode->m_rightSubTree);
    }
    return 0;
}

int cCollisionOBB::getTreeDepth(cCollisionOBBNode* a_node) const
{
    if (a_node == nullptr) return 0;
    if (a_node->m_nodeType == C_COLLISION_OBB_NODE_LEAF) return 1;

    cCollisionOBBInternal* internalNode = dynamic_cast<cCollisionOBBInternal*>(a_node);
    if (internalNode != nullptr)
    {
        return 1 + std::max(getTreeDepth(internalNode->m_leftSubTree), 
                            getTreeDepth(internalNode->m_rightSubTree));
    }
    return 0;
}

bool cCollisionOBB::computeCollision(cGenericObject* a_object,
                                      cVector3d& a_segmentPointA,
                                      cVector3d& a_segmentPointB,
                                      cCollisionRecorder& a_recorder,
                                      cCollisionSettings& a_settings)
{
    // Se cai dat o Giai doan 4[cite: 1, 4]
    return false;
}

void cCollisionOBB::render(cRenderOptions& a_options)
{
    // Se cai dat o Giai doan 5[cite: 1, 4]
}

} // namespace chai3d
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
    if (!m_triangles.empty())
    {
        initialize(m_triangles);
    }
}

void cCollisionOBB::initialize(const std::vector<cTriangle*>& a_triangles)
{
    if (m_root != nullptr)
    {
        delete m_root;
        m_root = nullptr;
    }

    m_triangles = a_triangles;
    if (m_triangles.empty()) return;

    std::vector<cTriangle*> triList = m_triangles;
    m_root = buildTree(triList, 0);
}

cCollisionOBBNode* cCollisionOBB::buildTree(std::vector<cTriangle*>& a_triangles, int a_depth)
{
    if (a_triangles.empty()) return nullptr;

    // 1. Thu thap toan bo dinh tu cac tam giac
    std::vector<cVector3d> points;
    points.reserve(a_triangles.size() * 3);
    for (const auto& tri : a_triangles)
    {
        points.push_back(tri->m_v0);
        points.push_back(tri->m_v1);
        points.push_back(tri->m_v2);
    }

    // 2. Tinh toan OBB toi uu bang PCA
    cCollisionOBBBox nodeBox;
    buildOBBFromPoints(points, nodeBox);

    // 3. BASE CASE: Node la (<= 1 tam giac)
    if (a_triangles.size() <= 1)
    {
        cCollisionOBBLeaf* leaf = new cCollisionOBBLeaf(a_triangles[0]);
        leaf->m_bbox = nodeBox;
        return leaf;
    }

    // 4. Tim truc dai nhat cua OBB
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

    // 5. Phan loai tam giac vao nhanh trai va phai
    std::vector<cTriangle*> leftList;
    std::vector<cTriangle*> rightList;

    for (const auto& tri : a_triangles)
    {
        cVector3d centroid = tri->computeCentroid();
        cVector3d diff = centroid - splitOrigin;

        double proj = diff.x() * splitAxis.x() + diff.y() * splitAxis.y() + diff.z() * splitAxis.z();

        if (proj >= 0.0)
            leftList.push_back(tri);
        else
            rightList.push_back(tri);
    }

    // 6. Xu ly suy bien bang Median Split
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

    // 7. Khoi tao node trung gian va de quy dung 2 cay con
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
    if (m_root == nullptr) return false;

    bool hit = m_root->computeCollision(a_object, a_segmentPointA, a_segmentPointB, a_recorder, a_settings);

    if (hit && !a_recorder.m_collisions.empty())
    {
        std::sort(a_recorder.m_collisions.begin(), a_recorder.m_collisions.end(),
            [](const cCollisionEvent& a, const cCollisionEvent& b) {
                return a.m_squareDistance < b.m_squareDistance;
            });
    }

    return hit;
}

void cCollisionOBB::render(cRenderOptions& a_options)
{
}

} // namespace chai3d
#include "CCollisionOBB.h"
#include "OBBMath.h"
#include "graphics/COpenGLHeaders.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <string>

namespace chai3d {

cCollisionOBB::cCollisionOBB()
{
    m_root = nullptr;
    m_useNeighbors = true;
    m_lastCollidedTriangle = nullptr;
    m_showBoundingBoxes = true;
    m_displayDepth = -1;
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

void cCollisionOBB::buildNeighbors()
{
    // Tạo khóa xác định một cạnh nối giữa 2 đỉnh trong không gian
    auto edgeKey = [](const cVector3d& p1, const cVector3d& p2) {
        cVector3d a = p1;
        cVector3d b = p2;
        if (a.x() > b.x() || 
           (a.x() == b.x() && a.y() > b.y()) || 
           (a.x() == b.x() && a.y() == b.y() && a.z() > b.z()))
        {
            std::swap(a, b);
        }
        return std::to_string(std::round(a.x() * 1e4)) + "_" +
               std::to_string(std::round(a.y() * 1e4)) + "_" +
               std::to_string(std::round(a.z() * 1e4)) + "#" +
               std::to_string(std::round(b.x() * 1e4)) + "_" +
               std::to_string(std::round(b.y() * 1e4)) + "_" +
               std::to_string(std::round(b.z() * 1e4));
    };

    std::map<std::string, std::vector<cTriangle*>> edgeMap;

    for (auto tri : m_triangles)
    {
        tri->m_neighbors.clear();
        edgeMap[edgeKey(tri->m_v0, tri->m_v1)].push_back(tri);
        edgeMap[edgeKey(tri->m_v1, tri->m_v2)].push_back(tri);
        edgeMap[edgeKey(tri->m_v2, tri->m_v0)].push_back(tri);
    }

    auto addNeighborPair = [](cTriangle* t1, cTriangle* t2) {
        if (t1 != t2)
        {
            if (std::find(t1->m_neighbors.begin(), t1->m_neighbors.end(), t2) == t1->m_neighbors.end())
                t1->m_neighbors.push_back(t2);
            if (std::find(t2->m_neighbors.begin(), t2->m_neighbors.end(), t1) == t2->m_neighbors.end())
                t2->m_neighbors.push_back(t1);
        }
    };

    for (const auto& pair : edgeMap)
    {
        const auto& list = pair.second;
        if (list.size() > 1)
        {
            for (size_t i = 0; i < list.size(); ++i)
            {
                for (size_t j = i + 1; j < list.size(); ++j)
                {
                    addNeighborPair(list[i], list[j]);
                }
            }
        }
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
    m_lastCollidedTriangle = nullptr;
    if (m_triangles.empty()) return;

    std::vector<cTriangle*> triList = m_triangles;
    m_root = buildTree(triList, 0);

    // Xây dựng danh sách tam giác láng giềng cho Local Search
    buildNeighbors();
}

cCollisionOBBNode* cCollisionOBB::buildTree(std::vector<cTriangle*>& a_triangles, int a_depth)
{
    if (a_triangles.empty()) return nullptr;

    std::vector<cVector3d> points;
    points.reserve(a_triangles.size() * 3);
    for (const auto& tri : a_triangles)
    {
        points.push_back(tri->m_v0);
        points.push_back(tri->m_v1);
        points.push_back(tri->m_v2);
    }

    cCollisionOBBBox nodeBox;
    buildOBBFromPoints(points, nodeBox);

    if (a_triangles.size() <= 1)
    {
        cCollisionOBBLeaf* leaf = new cCollisionOBBLeaf(a_triangles[0]);
        leaf->m_bbox = nodeBox;
        return leaf;
    }

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

    // 1. LOCAL SEARCH: Ưu tiên kiểm tra tam giác va chạm trước đó và các láng giềng
    if (m_useNeighbors && m_lastCollidedTriangle != nullptr)
    {
        cVector3d hitPoint, hitNormal;
        double tHit = 0.0;

        if (m_lastCollidedTriangle->computeCollision(a_segmentPointA, a_segmentPointB, hitPoint, hitNormal, tHit))
        {
            cCollisionEvent event;
            event.m_object = a_object;
            event.m_index = m_lastCollidedTriangle->m_index;
            event.m_localPos = hitPoint;
            event.m_globalPos = hitPoint;
            event.m_localNormal = hitNormal;
            event.m_globalNormal = hitNormal;
            event.m_squareDistance = (hitPoint - a_segmentPointA).lengthsq();
            a_recorder.m_collisions.push_back(event);
            return true;
        }

        for (auto neighbor : m_lastCollidedTriangle->m_neighbors)
        {
            if (neighbor->computeCollision(a_segmentPointA, a_segmentPointB, hitPoint, hitNormal, tHit))
            {
                cCollisionEvent event;
                event.m_object = a_object;
                event.m_index = neighbor->m_index;
                event.m_localPos = hitPoint;
                event.m_globalPos = hitPoint;
                event.m_localNormal = hitNormal;
                event.m_globalNormal = hitNormal;
                event.m_squareDistance = (hitPoint - a_segmentPointA).lengthsq();
                a_recorder.m_collisions.push_back(event);

                m_lastCollidedTriangle = neighbor;
                return true;
            }
        }
    }

    // 2. FALLBACK: Duyệt DFS toàn bộ cây OBB nếu Local Search trượt
    bool hit = m_root->computeCollision(a_object, a_segmentPointA, a_segmentPointB, a_recorder, a_settings);

    if (hit && !a_recorder.m_collisions.empty())
    {
        std::sort(a_recorder.m_collisions.begin(), a_recorder.m_collisions.end(),
            [](const cCollisionEvent& a, const cCollisionEvent& b) {
                return a.m_squareDistance < b.m_squareDistance;
            });

        int hitIndex = a_recorder.m_collisions[0].m_index;
        if (hitIndex >= 0 && hitIndex < (int)m_triangles.size())
        {
            m_lastCollidedTriangle = m_triangles[hitIndex];
        }
    }
    else
    {
        m_lastCollidedTriangle = nullptr;
    }

    return hit;
}

void cCollisionOBB::render(cRenderOptions& a_options)
{
    if (!m_showBoundingBoxes || m_root == nullptr) return;

    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT);
    glDisable(GL_LIGHTING);
    glLineWidth(1.5f);
    glColor3f(0.0f, 1.0f, 0.8f);

    m_root->render(a_options, 0, m_displayDepth);

    glPopAttrib();
}

} // namespace chai3d
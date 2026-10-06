#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include "chai3d.h"
#include "CCollisionOBB.h"

using namespace chai3d;

void runBenchmark()
{
    // Use increasingly dense spheres to compare build and coherent-query cost.
    std::cout << "\n===============================================================\n";
    std::cout << "                 CHAI3D COLLISION BENCHMARK                    \n";
    std::cout << "           So sanh: AABB Tree vs OBB Tree vs OBB Local Search  \n";
    std::cout << "===============================================================\n";

    int slicesArray[] = {16, 45, 130}; // Tao ra cac mesh xap xi 1k, 10k, 100k tam giac

    for (int slices : slicesArray)
    {
        cMesh* mesh = new cMesh();
        cCreateSphere(mesh, 1.0, slices, slices);
        int numTriangles = mesh->getNumTriangles();

        std::vector<cTriangle*> triList;
        triList.reserve(numTriangles);
        for (int i = 0; i < numTriangles; ++i)
        {
            cVector3d v0 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex0(i));
            cVector3d v1 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex1(i));
            cVector3d v2 = mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex2(i));
            triList.push_back(new cTriangle(v0, v1, v2, i));
        }

        std::cout << "\n--- Mesh Complexity: " << numTriangles << " Triangles ---\n";

        // 1. Benchmark Thoi gian xay cay (Build Time)
        auto start = std::chrono::high_resolution_clock::now();
        mesh->createAABBCollisionDetector(0.0);
        auto end = std::chrono::high_resolution_clock::now();
        double aabbBuildTime = std::chrono::duration<double, std::milli>(end - start).count();

        cCollisionOBB obbDetector;
        start = std::chrono::high_resolution_clock::now();
        obbDetector.initialize(triList);
        end = std::chrono::high_resolution_clock::now();
        double obbBuildTime = std::chrono::duration<double, std::milli>(end - start).count();

        std::cout << "Build Time: AABB = " << std::fixed << std::setprecision(2) << aabbBuildTime 
                  << " ms | OBB = " << obbBuildTime << " ms\n";

        // 2. Benchmark Truy van va cham lien tuc (5,000 queries mo phong chuyen dong HIP)
        const int numQueries = 5000;
        std::vector<std::pair<cVector3d, cVector3d>> coherentRays;
        cVector3d currentHip(1.1, 0.0, 0.0);

        for (int q = 0; q < numQueries; ++q)
        {
            currentHip += cVector3d(0.0002, 0.0003, 0.0001);
            coherentRays.push_back({currentHip, currentHip - cVector3d(0.3, 0.0, 0.0)});
        }

        cCollisionRecorder recorder;
        cCollisionSettings settings;

        // Test AABB Tree
        start = std::chrono::high_resolution_clock::now();
        for (const auto& ray : coherentRays)
        {
            recorder.m_collisions.clear();
            cVector3d pA = ray.first;
            cVector3d pB = ray.second;
            mesh->computeCollisionDetection(pA, pB, recorder, settings);
        }
        end = std::chrono::high_resolution_clock::now();
        double aabbQueryTime = std::chrono::duration<double, std::micro>(end - start).count() / numQueries;

        // Test OBB Tree (Khong bat Local Search)
        obbDetector.m_useNeighbors = false;
        start = std::chrono::high_resolution_clock::now();
        for (const auto& ray : coherentRays)
        {
            recorder.m_collisions.clear();
            cVector3d pA = ray.first;
            cVector3d pB = ray.second;
            obbDetector.computeCollision(mesh, pA, pB, recorder, settings);
        }
        end = std::chrono::high_resolution_clock::now();
        double obbQueryTime = std::chrono::duration<double, std::micro>(end - start).count() / numQueries;

        // Test OBB Tree (Co bat Local Search)
        obbDetector.m_useNeighbors = true;
        obbDetector.m_lastCollidedTriangle = nullptr;
        start = std::chrono::high_resolution_clock::now();
        for (const auto& ray : coherentRays)
        {
            recorder.m_collisions.clear();
            cVector3d pA = ray.first;
            cVector3d pB = ray.second;
            obbDetector.computeCollision(mesh, pA, pB, recorder, settings);
        }
        end = std::chrono::high_resolution_clock::now();
        double obbLocalQueryTime = std::chrono::duration<double, std::micro>(end - start).count() / numQueries;

        std::cout << "Avg Query Time: AABB = " << aabbQueryTime << " us\n"
                  << "                OBB (DFS) = " << obbQueryTime << " us\n"
                  << "                OBB (Local Search) = " << obbLocalQueryTime << " us\n";

        for (auto tri : triList) delete tri;
        delete mesh;
    }

    std::cout << "\n===============================================================\n\n";
}
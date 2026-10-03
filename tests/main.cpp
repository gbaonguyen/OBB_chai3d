#include <iostream>
#include <vector>
#include <cassert>
#include "chai3d.h"
#include <GLFW/glfw3.h>
#include "CCollisionOBBBox.h"
#include "CCollisionOBBNode.h"
#include "CCollisionOBBInternal.h"
#include "CCollisionOBBLeaf.h"
#include "CCollisionOBB.h"

void runMathTests();
void runTraversalTests();
void runTreeTests();
void runBenchmark(); // <-- Them khai bao nay

using namespace chai3d;

// Bien toan cuc quan ly camera va dieu khien
cWorld* world = nullptr;
cCamera* camera = nullptr;
cDirectionalLight* light = nullptr;
cMesh* meshObject = nullptr;
cCollisionOBB* obbDetector = nullptr;

int g_maxTreeDepth = 0;
double g_cameraAngleH = 0.5;
double g_cameraAngleV = 0.3;
double g_cameraDistance = 2.5;
bool g_mouseGrabbed = false;
double g_mouseX = 0.0;
double g_mouseY = 0.0;

// Xu ly su kien ban phim
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    if (key == GLFW_KEY_ESCAPE)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    else if (key == GLFW_KEY_B)
    {
        obbDetector->m_showBoundingBoxes = !obbDetector->m_showBoundingBoxes;
        meshObject->setShowCollisionDetector(obbDetector->m_showBoundingBoxes);
        std::cout << "[Visualizer] OBB Visibility: " 
                << (obbDetector->m_showBoundingBoxes ? "ON" : "OFF") << std::endl;
    }
    else if (key == GLFW_KEY_UP)
    {
        int cur = obbDetector->getDisplayDepth();
        if (cur < g_maxTreeDepth)
        {
            obbDetector->setDisplayDepth(cur + 1);
            std::cout << "[Visualizer] Display Depth Level: " << obbDetector->getDisplayDepth() << std::endl;
        }
    }
    else if (key == GLFW_KEY_DOWN)
    {
        int cur = obbDetector->getDisplayDepth();
        if (cur > -1)
        {
            obbDetector->setDisplayDepth(cur - 1);
            if (obbDetector->getDisplayDepth() == -1)
                std::cout << "[Visualizer] Display Depth Level: ALL LEVELS (-1)" << std::endl;
            else
                std::cout << "[Visualizer] Display Depth Level: " << obbDetector->getDisplayDepth() << std::endl;
        }
    }
}

// Xu ly xoay camera bang chuot
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        g_mouseGrabbed = (action == GLFW_PRESS);
        glfwGetCursorPos(window, &g_mouseX, &g_mouseY);
    }
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (g_mouseGrabbed)
    {
        double dx = xpos - g_mouseX;
        double dy = ypos - g_mouseY;
        g_mouseX = xpos;
        g_mouseY = ypos;

        g_cameraAngleH -= dx * 0.005;
        g_cameraAngleV += dy * 0.005;

        if (g_cameraAngleV > 1.5) g_cameraAngleV = 1.5;
        if (g_cameraAngleV < -1.5) g_cameraAngleV = -1.5;

        cVector3d camPos(
            g_cameraDistance * std::cos(g_cameraAngleV) * std::sin(g_cameraAngleH),
            g_cameraDistance * std::cos(g_cameraAngleV) * std::cos(g_cameraAngleH),
            g_cameraDistance * std::sin(g_cameraAngleV)
        );
        camera->set(camPos, cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 1.0));
    }
}

// Chay Visualizer Demo
void runVisualizationDemo()
{
    std::cout << "\n===================================================\n";
    std::cout << "KHOI DONG OBB VISUALIZATION WINDOW (OPENGL / GLFW)\n";
    std::cout << "Phim 'B'       : Bat / Tat hien thi OBB\n";
    std::cout << "Phim 'UP'/'DOWN': Tang / Giam tang do sau cay OBB\n";
    std::cout << "Keo chuot trai : Xoay goc nhin Camera\n";
    std::cout << "Phim 'ESC'     : Thoat\n";
    std::cout << "===================================================\n\n";

    if (!glfwInit())
    {
        std::cerr << "Khong the khoi tao GLFW!\n";
        return;
    }

    const int width = 1024;
    const int height = 768;
    GLFWwindow* window = glfwCreateWindow(width, height, "CHAI3D - OBB Tree Visualization", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);

    // 1. Khoi tao CHAI3D World, Camera, Lighting
    world = new cWorld();
    world->m_backgroundColor.setBlack();

    camera = new cCamera(world);
    world->addChild(camera);
    camera->set(cVector3d(1.8, 1.8, 1.2), cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 1.0));
    camera->setClippingPlanes(0.1, 10.0);

    light = new cDirectionalLight(world);
    world->addChild(light);
    light->setEnabled(true);
    light->setDir(-1.0, -1.0, -1.0);

// 2. Tao hinh 3D mau: Hinh cau gom nhieu tam giac cong
    meshObject = new cMesh();
    world->addChild(meshObject);
    cCreateSphere(meshObject, 0.5, 32, 32); // Su dung cCreateSphere thay cho cCreateTorus
    meshObject->m_material->setOrangeCoral();

    // 3. Trich xuat cac tam giac tu cMesh va khoi tao cCollisionOBB
    std::vector<cTriangle*> triList;
    triList.reserve(meshObject->getNumTriangles());

    for (int i = 0; i < meshObject->getNumTriangles(); ++i)
    {
        cVector3d v0 = meshObject->m_vertices->getLocalPos(meshObject->m_triangles->getVertexIndex0(i));
        cVector3d v1 = meshObject->m_vertices->getLocalPos(meshObject->m_triangles->getVertexIndex1(i));
        cVector3d v2 = meshObject->m_vertices->getLocalPos(meshObject->m_triangles->getVertexIndex2(i));
        triList.push_back(new cTriangle(v0, v1, v2, i));
    }

    obbDetector = new cCollisionOBB();
    obbDetector->initialize(triList);

    // Su dung setter hop le cua Chai3D thay vi gan truc tiep bien protected
    meshObject->setCollisionDetector(obbDetector);
    meshObject->setShowCollisionDetector(true);

    g_maxTreeDepth = obbDetector->getTreeDepth(obbDetector->m_root);
    std::cout << "[Mesh Info] Tong so tam giac: " << triList.size() << std::endl;
    std::cout << "[Tree Info] Do sau toi da cua cay OBB: " << g_maxTreeDepth << " levels\n";

    // 4. Vong lap Render chinh
    while (!glfwWindowShouldClose(window))
    {
        int fWidth, fHeight;
        glfwGetFramebufferSize(window, &fWidth, &fHeight);
        glViewport(0, 0, fWidth, fHeight);

        // Render canh CHAI3D va khung day OBB thong qua camera
        camera->renderView(fWidth, fHeight);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Don dep bo nho
    for (auto tri : triList) delete tri;
    delete world;

    glfwDestroyWindow(window);
    glfwTerminate();
}

void runTreeTests()
{
    std::cout << "\n--- BAT DAU KIEM TRA GIAI DOAN 3 (TOP-DOWN TREE) ---\n";
    cVector3d p0( 1.0,  1.0,  1.0);
    cVector3d p1(-1.0, -1.0,  1.0);
    cVector3d p2(-1.0,  1.0, -1.0);
    cVector3d p3( 1.0, -1.0, -1.0);

    std::vector<cTriangle*> triangles;
    triangles.push_back(new cTriangle(p0, p1, p2, 0));
    triangles.push_back(new cTriangle(p0, p1, p3, 1));
    triangles.push_back(new cTriangle(p0, p2, p3, 2));
    triangles.push_back(new cTriangle(p1, p2, p3, 3));

    cCollisionOBB obbTree;
    obbTree.initialize(triangles);

    assert(obbTree.m_root != nullptr);
    int leafCount = obbTree.getLeafCount(obbTree.m_root);
    assert(leafCount == 4);
    int depth = obbTree.getTreeDepth(obbTree.m_root);
    assert(depth >= 2 && depth <= 4);

    for (auto tri : triangles) delete tri;
    std::cout << "[PASS] Cay Top-Down phan tach hop le.\n";
    std::cout << "--- HOAN THANH GIAI DOAN 3 THANH CONG! ---\n\n";
}

int main()
{
    std::cout << "=== CHAY TOAN BO UNIT TESTS CAC GIAI DOAN ===\n";

    // Kiem tra Phase 1
    cCollisionOBBBox box;
    box.m_center.set(2.0, 3.0, 4.0);
    box.m_extent.set(0.5, 1.0, 1.5);
    assert(box.m_center.x() == 2.0);
    std::cout << "[PASS] Giai doan 1 (Data Structures) OK.\n";

    // Kiem tra Phase 2
    runMathTests();

    // Kiem tra Phase 3
    runTreeTests();

    // Kiem tra Phase 4
    runTraversalTests();

    std::cout << "=== TOAN BO UNIT TESTS DA VUOT QUA THANH CONG! ===\n";

    // Chay Benchmark Giai doan 6 (Hang muc 2)
    runBenchmark();

    // Khoi chay truc quan hoa Giai doan 5
    runVisualizationDemo();

    return 0;
}
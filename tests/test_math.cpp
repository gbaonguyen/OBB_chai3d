#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "OBBMath.h"

using namespace chai3d;

void runMathTests()
{
    std::cout << "\n--- BAT DAU KIEM TRA GIAI DOAN 2 (PCA & JACOBI) ---\n";

    // Tao 8 dinh cua hop chu nhat nghieng 45 do quanh truc Z
    // Kich thuoc ban dau: x trong [-2, 2], y trong [-1, 1], z trong [-0.5, 0.5]
    // Nua chieu dai (extents) ky vong: [2.0, 1.0, 0.5]
    double cos45 = std::cos(M_PI / 4.0);
    double sin45 = std::sin(M_PI / 4.0);

    std::vector<cVector3d> points;
    double xs[2] = {-2.0, 2.0};
    double ys[2] = {-1.0, 1.0};
    double zs[2] = {-0.5, 0.5};

    for (double x : xs)
    {
        for (double y : ys)
        {
            for (double z : zs)
            {
                // Xoay 45 do quanh Z: x' = x*cos - y*sin; y' = x*sin + y*cos
                double rx = x * cos45 - y * sin45;
                double ry = x * sin45 + y * cos45;
                double rz = z;
                points.push_back(cVector3d(rx, ry, rz));
            }
        }
    }

    // 1. Tinh OBB tu tap diem
    cCollisionOBBBox box;
    buildOBBFromPoints(points, box);

    // 2. Kiem tra tinh truc chuan cua 3 vector truc u[0], u[1], u[2]
    // Sửa cDot(...) thành .dot()
    double dot01 = std::fabs(box.u[0].dot(box.u[1]));
    double dot02 = std::fabs(box.u[0].dot(box.u[2]));
    double dot12 = std::fabs(box.u[1].dot(box.u[2]));
    assert(dot01 < 1e-5 && "u[0] va u[1] khong vuong goc!");
    assert(dot02 < 1e-5 && "u[0] va u[2] khong vuong goc!");
    assert(dot12 < 1e-5 && "u[1] va u[2] khong vuong goc!");
    std::cout << "[PASS] 3 truc toa do u[0], u[1], u[2] truc giao hoan toan.\n";

    // 3. Kiem tra do dai nua truc (Extents)
    // Truc dai nhat phai xap xi 2.0, truc nhi xap xi 1.0, truc ngan nhat xap xi 0.5
    assert(std::fabs(box.m_extent.x() - 2.0) < 1e-4 && "Extent X tinh sai!");
    assert(std::fabs(box.m_extent.y() - 1.0) < 1e-4 && "Extent Y tinh sai!");
    assert(std::fabs(box.m_extent.z() - 0.5) < 1e-4 && "Extent Z tinh sai!");
    std::cout << "[PASS] Extents tinh chinh xac: (" 
              << box.m_extent.x() << ", " 
              << box.m_extent.y() << ", " 
              << box.m_extent.z() << ")\n";

    // 4. Kiem tra toa do tam
    assert(box.m_center.length() < 1e-4 && "Center khong trung goc toa do!");
    std::cout << "[PASS] Center tinh chinh xac tai goc toa do (0, 0, 0).\n";

    std::cout << "--- HOAN THANH GIAI DOAN 2 THANH CONG! ---\n\n";
}
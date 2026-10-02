#ifndef OBB_MATH_H
#define OBB_MATH_H

#include <vector>
#include "math/CVector3d.h"
#include "CCollisionOBBBox.h"

namespace chai3d {

// 1. Tinh trong tam cua tap diem
cVector3d computeCentroid(const std::vector<cVector3d>& a_points);

// 2. Xay dung ma tran hiep phuong sai 3x3
void computeCovarianceMatrix(const std::vector<cVector3d>& a_points, 
                             const cVector3d& a_centroid, 
                             double a_cov[3][3]);

// 3. Tim tri rieng va vector rieng tu ma tran 3x3
void computeEigenSystem(double a_cov[3][3], 
                        double a_eigenvalues[3], 
                        cVector3d a_eigenvectors[3]);

// 4. Chieu tap diem len truc de tim gia tri min/max
void computeExtremePointsAlongDirection(const std::vector<cVector3d>& a_points,
                                        const cVector3d& a_direction,
                                        double& a_min,
                                        double& a_max);

// 5. Ham tong hop: Tinh toan hoan chinh hop OBB tu tap diem
void buildOBBFromPoints(const std::vector<cVector3d>& a_points, 
                        cCollisionOBBBox& a_box);

} // namespace chai3d

#endif // OBB_MATH_H
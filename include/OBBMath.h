#ifndef OBB_MATH_H
#define OBB_MATH_H

#include <vector>
#include "math/CVector3d.h"
#include "CCollisionOBBBox.h"

namespace chai3d {

cVector3d computeCentroid(const std::vector<cVector3d>& a_points);

void computeCovarianceMatrix(const std::vector<cVector3d>& a_points, 
                             const cVector3d& a_centroid, 
                             double a_cov[3][3]);

void computeEigenSystem(double a_cov[3][3], 
                        double a_eigenvalues[3], 
                        cVector3d a_eigenvectors[3]);

void computeExtremePointsAlongDirection(const std::vector<cVector3d>& a_points,
                                        const cVector3d& a_direction,
                                        double& a_min,
                                        double& a_max);

void buildOBBFromPoints(const std::vector<cVector3d>& a_points, 
                        cCollisionOBBBox& a_box);

} // namespace chai3d

#endif // OBB_MATH_H
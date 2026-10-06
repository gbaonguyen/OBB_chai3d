#include "OBBMath.h"
#include <cmath>
#include <algorithm>

namespace chai3d {

// Return the arithmetic mean; an empty point set maps to the origin.
cVector3d computeCentroid(const std::vector<cVector3d>& a_points)
{
    cVector3d centroid(0.0, 0.0, 0.0);
    if (a_points.empty()) return centroid;

    for (const auto& p : a_points)
    {
        centroid = centroid + p;
    }

    double invN = 1.0 / static_cast<double>(a_points.size());
    return centroid * invN;
}

void computeCovarianceMatrix(const std::vector<cVector3d>& a_points, 
                             const cVector3d& a_centroid, 
                             double a_cov[3][3])
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            a_cov[i][j] = 0.0;
        }
    }

    if (a_points.empty()) return;

    for (const auto& p : a_points)
    {
        cVector3d d = p - a_centroid;
        double coords[3] = { d.x(), d.y(), d.z() };

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                a_cov[i][j] += coords[i] * coords[j];
            }
        }
    }

    double invN = 1.0 / static_cast<double>(a_points.size());
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            a_cov[i][j] *= invN;
        }
    }
}

void computeEigenSystem(double a_cov[3][3], 
                        double a_eigenvalues[3], 
                        cVector3d a_eigenvectors[3])
{
    // Jacobi rotations diagonalize the symmetric 3x3 covariance matrix.
    double A[3][3];
    double V[3][3];

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            A[i][j] = a_cov[i][j];
            V[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    const int maxIterations = 50;
    const double eps = 1e-10;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        int p = 0, q = 1;
        double maxVal = std::fabs(A[0][1]);
        if (std::fabs(A[0][2]) > maxVal) { maxVal = std::fabs(A[0][2]); p = 0; q = 2; }
        if (std::fabs(A[1][2]) > maxVal) { maxVal = std::fabs(A[1][2]); p = 1; q = 2; }

        if (maxVal < eps) break;

        double app = A[p][p];
        double aqq = A[q][q];
        double apq = A[p][q];

        double phi = 0.5 * (aqq - app) / apq;
        double t = (phi >= 0.0) ? (1.0 / (phi + std::sqrt(phi * phi + 1.0)))
                                : (-1.0 / (-phi + std::sqrt(phi * phi + 1.0)));
        double c = 1.0 / std::sqrt(t * t + 1.0);
        double s = t * c;
        double tau = s / (1.0 + c);

        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = 0.0;
        A[q][p] = 0.0;

        for (int r = 0; r < 3; ++r)
        {
            if (r != p && r != q)
            {
                double arp = A[r][p];
                double arq = A[r][q];
                A[r][p] = A[p][r] = arp - s * (arq + tau * arp);
                A[r][q] = A[q][r] = arq + s * (arp - tau * arq);
            }
        }

        for (int r = 0; r < 3; ++r)
        {
            double vrp = V[r][p];
            double vrq = V[r][q];
            V[r][p] = vrp - s * (vrq + tau * vrp);
            V[r][q] = vrq + s * (vrp - tau * vrq);
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        a_eigenvalues[i] = A[i][i];
        a_eigenvectors[i].set(V[0][i], V[1][i], V[2][i]);
        a_eigenvectors[i].normalize();
    }

    for (int i = 0; i < 2; ++i)
    {
        for (int j = i + 1; j < 3; ++j)
        {
            if (a_eigenvalues[j] > a_eigenvalues[i])
            {
                std::swap(a_eigenvalues[i], a_eigenvalues[j]);
                std::swap(a_eigenvectors[i], a_eigenvectors[j]);
            }
        }
    }

    cVector3d crossCheck(
        a_eigenvectors[0].y() * a_eigenvectors[1].z() - a_eigenvectors[0].z() * a_eigenvectors[1].y(),
        a_eigenvectors[0].z() * a_eigenvectors[1].x() - a_eigenvectors[0].x() * a_eigenvectors[1].z(),
        a_eigenvectors[0].x() * a_eigenvectors[1].y() - a_eigenvectors[0].y() * a_eigenvectors[1].x()
    );

    double dotProduct = crossCheck.x() * a_eigenvectors[2].x() +
                        crossCheck.y() * a_eigenvectors[2].y() +
                        crossCheck.z() * a_eigenvectors[2].z();

    if (dotProduct < 0.0)
    {
        a_eigenvectors[2] = a_eigenvectors[2] * -1.0;
    }
}

void computeExtremePointsAlongDirection(const std::vector<cVector3d>& a_points,
                                        const cVector3d& a_direction,
                                        double& a_min,
                                        double& a_max)
{
    if (a_points.empty())
    {
        a_min = 0.0;
        a_max = 0.0;
        return;
    }

    auto dotProd = [](const cVector3d& v1, const cVector3d& v2) {
        return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
    };

    a_min = dotProd(a_points[0], a_direction);
    a_max = a_min;

    for (size_t i = 1; i < a_points.size(); ++i)
    {
        double proj = dotProd(a_points[i], a_direction);
        if (proj < a_min) a_min = proj;
        if (proj > a_max) a_max = proj;
    }
}

void buildOBBFromPoints(const std::vector<cVector3d>& a_points, cCollisionOBBBox& a_box)
{
    if (a_points.empty()) return;

    // Principal directions provide the box frame; projections provide its size.
    cVector3d centroid = computeCentroid(a_points);

    double cov[3][3];
    computeCovarianceMatrix(a_points, centroid, cov);

    double eigenvalues[3];
    computeEigenSystem(cov, eigenvalues, a_box.u);

    cVector3d center(0.0, 0.0, 0.0);
    double extentArr[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; ++i)
    {
        double minVal = 0.0;
        double maxVal = 0.0;
        computeExtremePointsAlongDirection(a_points, a_box.u[i], minVal, maxVal);

        extentArr[i] = 0.5 * (maxVal - minVal);
        double midVal = 0.5 * (maxVal + minVal);

        center = center + (a_box.u[i] * midVal);
    }

    a_box.m_center = center;
    a_box.m_extent.set(extentArr[0], extentArr[1], extentArr[2]);
}

} // namespace chai3d
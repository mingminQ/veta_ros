/**
 * --------------------------------------------------
 *
 * @file    CubisSpline.h
 * @brief   Cubic spline solver utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/math/polynomial/CubicSpline.h"
#include "veta/util/LogHandler.h"

#include <Eigen/LU>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

#include <numeric>
#include <cmath>

namespace // anonymous
{
    /** @brief Matrix A for coefficient solver
     * @details Return Eigen library data type, private method is recommended
     */
    Eigen::MatrixXd getMatrixA(const std::vector<double> &deltaX, const unsigned int &size)
    {
        Eigen::MatrixXd matrixA = Eigen::MatrixXd::Zero(size, size);
        matrixA(size - 1, size - 1) = 1.0;
        matrixA(0, 0) = 1.0;

        for(unsigned int idx = 1; idx < size - 1; idx++)
        {
            matrixA(idx, idx - 1) = deltaX[idx - 1];
            matrixA(idx, idx)     = 2.0 * (deltaX[idx - 1] + deltaX[idx]);
            matrixA(idx, idx + 1) = deltaX[idx];
        }

        return matrixA;
    }

    /** @brief Vector B for coefficient solver */
    Eigen::VectorXd getVectorB(const std::vector<double> &deltaX, const unsigned int &size, const std::vector<double> &coef0)
    {
        Eigen::VectorXd vectorB = Eigen::VectorXd::Zero(size);
        for(unsigned int idx = 1; idx < size - 1; idx++)
        {
            vectorB(idx) = 3.0 * ( (coef0[idx + 1] - coef0[idx]) / deltaX[idx] 
                        - (coef0[idx] - coef0[idx - 1]) / deltaX[idx - 1] );
        }

        return vectorB;
    }

} // namespace anonymous

// "veta::polynomial::CubicSpline1D" source

/** @brief Class constructor */
veta::polynomial::CubicSpline1D::CubicSpline1D(const std::vector<double> &vectorX, const std::vector<double> &vectorY)
  : m_size(vectorX.size())
  , m_vectorX(vectorX)
  , c0(vectorY)
{
    // @TODO Validation check

    // Calculate delta x
    std::vector<double> deltaX(m_size);
    std::adjacent_difference(vectorX.begin(), vectorX.end(), deltaX.begin());
    deltaX.erase(deltaX.begin());

    // Make matrix A and vector b
    c0 = vectorY;
    Eigen::MatrixXd matrixA = getMatrixA(deltaX, m_size);
    Eigen::VectorXd vectorB = getVectorB(deltaX, m_size, c0);

    // @TODO Solver algorithm upgrade
    // Sparse LU for performance
    Eigen::SparseMatrix<double> sparseMatrixA = matrixA.sparseView();
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(sparseMatrixA);

    // Solve coefficient order 2nd
    Eigen::VectorXd c2EigenType = solver.solve(vectorB);
    c2 = std::vector<double>(c2EigenType.data(), c2EigenType.data() + c2EigenType.size());

    // Solve coefficient order 1st and 3rd
    c1.resize(m_size - 1);
    c3.resize(m_size - 1);
    for(unsigned int idx = 0; idx < m_size - 1; idx++)
    {
        c3[idx] = (c2[idx + 1] - c2[idx]) / ( 3.0 * deltaX[idx] );
        c1[idx] = (c0[idx + 1] - c0[idx]) / deltaX[idx]
                - deltaX[idx] * ( 2.0 * c2[idx] + c2[idx + 1] ) / 3.0;
    }
}

/** @brief Index query by the nearest point */
unsigned int veta::polynomial::CubicSpline1D::getNearestIndex(const double &x) const
{
    auto iter = std::lower_bound(m_vectorX.begin(), m_vectorX.end(), x);
    if(iter == m_vectorX.begin())
    {
        return 0;
    }

    unsigned int idx = static_cast<unsigned int>(iter - m_vectorX.begin()) - 1;
    return idx;
}

/** @brief Position query by the nearest point */
double veta::polynomial::CubicSpline1D::getPosition(const double &x) const
{
    unsigned int idx = getNearestIndex(x);
    double dx = x - m_vectorX[idx];
    return c0[idx] + dx * (c1[idx] + dx * (c2[idx] + dx * c3[idx]));
}

/** @brief Derivation query by the nearest position */
double veta::polynomial::CubicSpline1D::getDerivation(const double &x, const unsigned int &order) const
{
    // @TODO Error check

    unsigned int idx = getNearestIndex(x);
    double dx = x - m_vectorX[idx];

    switch(order)
    {
        case(1):
            return c1[idx] + dx * (2.0 * c2[idx] + dx * 3.0 * c3[idx]);
        case(2):
            return 2.0 * c2[idx] + 6.0 * c3[idx];
        default:
            VETA_ERROR("Cubic Spline derivation order range is 1 to 2");
            return NAN;
    }
}

// "veta::polynomial::CubicSpline2D" source

/** @brief Class constructor */
veta::polynomial::CubicSpline2D::CubicSpline2D(const std::vector<double> &vectorX, const std::vector<double> &vectorY)
  : m_size(vectorX.size())
  , m_length(calculateLength(vectorX, vectorY))
  , m_cubicSplineX(std::make_unique<CubicSpline1D>(m_length, vectorX))
  , m_cubicSplineY(std::make_unique<CubicSpline1D>(m_length, vectorY))
{
    // @TODO Validation check
}

/** @brief Calculate the accumulative length of the polynomial */
std::vector<double> veta::polynomial::CubicSpline2D::calculateLength(const std::vector<double> &vectorX, const std::vector<double> &vectorY) const
{
    std::vector<double> dx(m_size - 1), dy(m_size - 1);
    std::transform(vectorX.begin() + 1, vectorX.end(), vectorX.begin(), dx.begin(), std::minus<double>());
    std::transform(vectorY.begin() + 1, vectorY.end(), vectorY.begin(), dy.begin(), std::minus<double>());

    std::vector<double> ds(m_size - 1);
    std::transform(dx.begin(), dx.end(), dy.begin(), ds.begin(), 
    [](double dx, double dy) 
    {
        return std::sqrt(dx * dx + dy * dy);
    });

    std::vector<double> arcLength(m_size);
    arcLength.front() = 0.0;
    std::partial_sum(ds.begin(), ds.end(), arcLength.begin() + 1);

    return arcLength;
}

/** @brief Accumulative legnth getter function */
std::vector<double> veta::polynomial::CubicSpline2D::getAccLength() const
{
    return m_length;
}

/** @brief Position query by the accumulative arc length */
veta::Point2D veta::polynomial::CubicSpline2D::getPosition(const double &s) const
{
    double x = m_cubicSplineX->getPosition(s);
    double y = m_cubicSplineY->getPosition(s);
    
    return Point2D(x, y);
}

/** @brief Heading angle query by the accumulative arc length */
double veta::polynomial::CubicSpline2D::getHeading(const double &s) const
{
    double dx = m_cubicSplineX->getDerivation(s, 1);
    double dy = m_cubicSplineY->getDerivation(s, 1);
    return std::atan2(dy, dx);
}

/** @brief Curvature query by the accumulative arc length */
double veta::polynomial::CubicSpline2D::getCurvature(const double &s) const
{
    double xDot = m_cubicSplineX->getDerivation(s, 1);
    double yDot = m_cubicSplineY->getDerivation(s, 1);

    double x2Dot = m_cubicSplineX->getDerivation(s, 2);
    double y2Dot = m_cubicSplineY->getDerivation(s, 2);

    double num = (y2Dot * xDot) - (x2Dot * yDot);
    double den = std::pow((xDot * xDot) + (yDot * yDot), 1.5);

    return num / den;
}
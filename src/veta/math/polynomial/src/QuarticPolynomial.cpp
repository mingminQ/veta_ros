/**
 * --------------------------------------------------
 *
 * @file    Quarticpolynomial.cpp
 * @brief   Quartic polynomial solver utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/math/polynomial/QuarticPolynomial.h"
#include "veta/util/LogHandler.h"

#include <Eigen/LU>
#include <cmath>

// "veta::polynomial::QuarticPolynomial" source

/** @brief Class constructor */
veta::polynomial::QuarticPolynomial::QuarticPolynomial(const double &xs , const double &vxs, const double &axs
                                                      ,const double &vxe, const double &axe, const double &timeDuration)
  : m_maxTime(timeDuration)
{
    solveCoefficients(xs, vxs, axs, vxe, axe, timeDuration);
}

/** @brief Quartic polynomial coefficient solver */
void veta::polynomial::QuarticPolynomial::solveCoefficients(const double &xs , const double &vxs, const double &axs
                                                           ,const double &vxe, const double &axe, const double &timeDuration)
{
    c0 = xs;
    c1 = vxs;
    c2 = axs / 2.0;

    // Matrix A and vector B to solve other coefficients
    double time2pow = timeDuration * timeDuration;
    double time3pow = time2pow * timeDuration;

    // Matrix A
    Eigen::Matrix2d matrixA;
    matrixA << ( 3.0 * time2pow)     , ( 4.0 * time3pow)
             , ( 6.0 * timeDuration ), (12.0 * time2pow);
    
    // Check singularity
    Eigen::FullPivLU<Eigen::Matrix2d> decompLU(matrixA);
    if(!decompLU.isInvertible())
    {
        VETA_ERROR("Matrix A is not invertible.");
        return;
    }
    
    // Vector B
    Eigen::Vector2d vectorB;
    vectorB << (vxe - c1 - 2.0 * c2 * timeDuration)
             , (axe - 2.0 * c2);

    // Solve
    Eigen::Vector2d solution = matrixA.lu().solve(vectorB);
    c3 = solution(0);
    c4 = solution(1);
}

/** @brief Position query by the nearest time */
double veta::polynomial::QuarticPolynomial::getPosition(const double &t) const
{
    if((t < 0.0) || (m_maxTime < t))
    {
        VETA_ERROR("Invalid time input (%lf), Valid range: [0.0, %lf]", t, m_maxTime);
        return NAN;
    }
    return c0 + t * (c1 + t * (c2 + t * (c3 + t * c4)));
}

/** @brief Derivation query by the nearest time */
double veta::polynomial::QuarticPolynomial::getDerivation(const double &t, const unsigned int &order) const
{
    if((t < 0.0) || (m_maxTime < t))
    {
        VETA_ERROR("Invalid time input (%lf), Valid range: [0.0, %lf]", t, m_maxTime);
        return NAN;
    }
    switch(order)
    {
        case(1):
            return c1 + t * (2.0 * c2 + t * (3.0 * c3 + t * (4.0 * c4)));
        case(2):
            return 2.0 * c2 + t * (6.0 * c3 + t * (12.0 * c4));
        case(3):
            return 6.0 * c3 + t * (24.0 * c4);

        default:
            VETA_ERROR("Quartic polynomial derivation order range is 1 to 3");
            return NAN;
    }
}
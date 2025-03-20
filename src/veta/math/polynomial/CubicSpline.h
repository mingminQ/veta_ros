/**
 * --------------------------------------------------
 *
 * @file    CubisSpline.h
 * @brief   Cubic spline solver utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_POLYNOMIAL_CUBICSPLINE_H_
#define VETA_MATH_POLYNOMIAL_CUBICSPLINE_H_

#include "veta/math/Point.h"

#include <memory>
#include <vector>

namespace veta
{
    namespace polynomial
    {   
        /** @details 1-Dimensional cubic spline polynomial solver */
        class CubicSpline1D
        {
        // "CubicSpline1D" member variables
        private:

            // Size of the cubic spline
            unsigned int m_size;

            // Vector storages
            std::vector<double> m_vectorX;

            // Cubic spline coefficients
            std::vector<double> c0, c1, c2, c3;

        // "CubicSpline1D" member functions
        private:

            /** @brief Index query by the nearest point */
            unsigned int getNearestIndex(const double &x) const;

        public:

            /** @brief Class constructor */
            CubicSpline1D(const std::vector<double> &vectorX, const std::vector<double> &vectorY);
            
            /** @brief Class destructor */
            ~CubicSpline1D() = default;

            /** @brief Position query by the nearest point */
            double getPosition(const double &x) const;

            /** @brief Derivation query by the nearest position */
            double getDerivation(const double &x, const unsigned int &order) const;

        }; // class CubicSpline1D

        /** @details 2-Dimensional cubic spline polynomial solver */
        class CubicSpline2D
        {
        // "CubicSpline2D" member variables
        private:

            // Size of the cubic spline
            unsigned int m_size;

            // Accumulative length of the each section
            std::vector<double> m_length;

            // Cubuc spline of the X and Y
            std::unique_ptr<CubicSpline1D> m_cubicSplineX;
            std::unique_ptr<CubicSpline1D> m_cubicSplineY;

        // "CubicSpline2D" member functions
        private:

            /** @brief Calculate the accumulative length of the polynomial */
            std::vector<double> calculateLength(const std::vector<double> &vectorX, const std::vector<double> &vectorY) const;

        public:

            /** @brief Class constructor */
            CubicSpline2D(const std::vector<double> &vectorX, const std::vector<double> &vectorY);

            /** @brief Class destructor */
            ~CubicSpline2D() = default;

            /** @brief Accumulative legnth getter function */
            std::vector<double> getAccLength() const;

            /** @brief Position query by the accumulative arc length */
            Point2D getPosition(const double &s) const;

            /** @brief Heading angle query by the accumulative arc length */
            double getHeading(const double &s) const;

            /** @brief Curvature query by the accumulative arc length */
            double getCurvature(const double &s) const;

    }; // class CubicSpline2D

    } // namespace polynomial

} // namespace veta

#endif // VETA_MATH_POLYNOMIAL_CUBICSPLINE_H_
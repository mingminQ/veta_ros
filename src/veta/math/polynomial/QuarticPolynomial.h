/**
 * --------------------------------------------------
 *
 * @file    QuarticPolynomial.h
 * @brief   Quartic polynomial solver utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_POLYNOMIAL_QUARTICPOLYNOMIAL_H_
#define VETA_MATH_POLYNOMIAL_QUARTICPOLYNOMIAL_H_

namespace veta
{
    namespace polynomial
    {
        /** @brief Quartic polynomial solver class */
        class QuarticPolynomial
        {
        // "QuarticPolynomial" member variables
        private:

            // Quartic polynomial coefficients
            double c0, c1, c2, c3, c4;

            // Sampling time
            double m_maxTime;

        // "QuinticPolynomial" member functions
        private:

            /** @brief Quartic polynomial coefficient solver */
            void solveCoefficients(const double &xs , const double &vxs, const double &axs
                                    ,const double &vxe, const double &axe, const double &timeDuration);

        public:

            /** @brief Class constructor */
            QuarticPolynomial(const double &xs , const double &vxs, const double &axs
                                ,const double &vxe, const double &axe, const double &timeDuration);

            /** @brief Class destructor */
            ~QuarticPolynomial() = default;

            /** @brief Position query by the nearest time */
            double getPosition(const double &t) const;

            /** @brief Derivation query by the nearest time */
            double getDerivation(const double &t, const unsigned int &order) const;

        }; // QuarticPolynomial

    } // namespace polynomial

} // namespace veta

#endif // VETA_MATH_POLYNOMIAL_QUARTICPOLYNOMIAL_H_
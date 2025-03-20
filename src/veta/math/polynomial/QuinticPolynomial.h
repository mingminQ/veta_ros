/**
 * --------------------------------------------------
 *
 * @file    QuinticPolynomial.h
 * @brief   Quintic polynomial solver utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_POLYNOMIAL_QUINTICPOLYNOMIAL_H_
#define VETA_MATH_POLYNOMIAL_QUINTICPOLYNOMIAL_H_

namespace veta
{
    namespace polynomial
    {
        /** @brief Quintic polynomial solver class */
        class QuinticPolynomial
        {
        // "QuinticPolynomial" member variables
        private:

            // Quintic polynomial coefficients
            double c0, c1, c2, c3, c4, c5;

            // Sampling time
            double m_maxTime;

        // "QuinticPolynomial" member functions
        private:

            /** @brief Quintic polynomial coefficient solver */
            void solveCoefficients(const double &xs, const double &vxs, const double &axs
                                    ,const double &xe, const double &vxe, const double &axe
                                    ,const double &timeDuration);

        public:

            /** @brief Class constructor */
            QuinticPolynomial(const double &xs, const double &vxs, const double &axs
                                ,const double &xe, const double &vxe, const double &axe
                                ,const double &timeDuration );

            /** @brief Class destructor */
            ~QuinticPolynomial() = default;

            /** @brief Position query by the nearest time */
            double getPosition(const double &t) const;

            /** @brief Derivation query by the nearest time */
            double getDerivation(const double &t, const unsigned int &order) const;

        }; // QuinticPolynomial

    } // namespace polynomial

} // namespace veta

#endif // VETA_MATH_POLYNOMIAL_QUINTICPOLYNOMIAL_H_
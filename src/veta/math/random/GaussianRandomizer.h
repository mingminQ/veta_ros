/**
 * --------------------------------------------------
 *
 * @file    GaussianRandomizer.h
 * @brief   Gaussian randomizer utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_RANDOM_GAUSSIANRANDOMIZER_H_
#define VETA_MATH_RANDOM_GAUSSIANRANDOMIZER_H_

#include "veta/math/random/Randomizer.h"

namespace veta
{
    namespace random
    {
        /** @brief Gaussian random number generator class */
        class GaussianRandomizer : public veta::random::Randomizer
        {
        // "GaussianRandomizer" member variables
        private:

            // @brief Normal gaussian probability distribution
            std::normal_distribution<> m_normalDist{0, 1};

        // "GaussianRandomizer" member functions
        public:

            /** @brief Class constructor */
            GaussianRandomizer();

            /** @brief Class constructor with initial local seed */
            GaussianRandomizer(std::uint_fast64_t localSeed);

            /** @brief Class destructor */
            ~GaussianRandomizer() override = default;

            /**
             * @brief Local random seed setter method and reset distribution
             * @param localSeed Local random seed
             */
            void setLocalSeed(std::uint_fast64_t localSeed) override;

            /**
             * @brief Real random number generation method by gaussian distribution
             * @param mean Mean of the gaussian distribution (default : 0.0)
             * @param stdDev Standard deviation of the gausian distribution (default : 1.0)
             */
            double gaussianReal(const double &mean = 0.0, const double &stdDev = 1.0);

            /**
             * @brief Real random number generation method by folded gaussian distribution
             * @param lowerBound Lower boundary of the result
             * @param upperBound Upper boundary of the result
             * @param bias Foucusing value around upper boundary
             */
            double foldedGaussianReal(const double &lowerBound = 0.0, const double &upperbound = 1.0, const double &bias = 1.0);

            /**
             * @brief Integer random number generation method by folded gaussian distribution
             * @param lowerBound Lower boundary of the result
             * @param upperBound Upper boundary of the result
             * @param bias Foucusing value around upper boundary
             */
            int foldedGaussianInt(const int &lowerBound, const int &upperBound, const double &bias = 1.0);

        }; // class GaussianRandomizer

    } // namespace random

} // namespace veta

#endif // VETA_MATH_RANDOM_GAUSSIANRANDOMIZER_H_
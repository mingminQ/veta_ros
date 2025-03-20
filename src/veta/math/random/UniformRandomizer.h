/**
 * --------------------------------------------------
 *
 * @file    UniformRandomizer.h
 * @brief   Uniform randomizer utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_RANDOM_UNIFORMRANDOMIZER_H_
#define VETA_UTIL_RANDOM_UNIFORMRANDOMIZER_H_

#include "veta/math/random/Randomizer.h"
// #include "veta/math/Angle.h"

namespace veta
{
    namespace random
    {
        /** @brief Uniform random number generator class */
        class UniformRandomizer : public veta::random::Randomizer
        {
        // "UniformRandomizer" member variables
        private:

            // Uniform probability distribution
            std::uniform_real_distribution<> m_uniformDist{0, 1};

        // "UniformRandomizer" member functions
        public:

            /** @brief Class constructor */
            UniformRandomizer();

            /** @brief Class constructor with initial local seed*/
            UniformRandomizer(std::uint_fast64_t localSeed);

            /** @brief Class destructor*/
            ~UniformRandomizer() override = default;

            /**
             * @brief Local random seed setter method and reset distribution
             * @param localSeed Local random seed
             */
            void setLocalSeed(std::uint_fast32_t localSeed) override;

            /**
             * @brief Real random number generation method by uniform distribution
             * @param lowerBound Lower boundary of the result (default : 0.0)
             * @param upperBound Upper boundary of the result (default : 1.0)
             */
            double uniformReal(double lowerBound = 0.0, double upperBound = 1.0);

            /**
             * @brief Integer random number generation method by uniform distribution
             * @param lowerBound Lower boundary of the result
             * @param upperBound Upper boundary of the result
             */
            int uniformInt(int lowerBound, int upperBound);

            /** @brief Boolean random number generation method by uniform distribution */
            bool uniformBool();

            // Quaternion uniformQuaternion();

            // EulerRPY uniformEulerRPY();

            // @TODO add sphere data uniform randomization

        }; // class UniformRandomizer

    } // namespace random
    
} // namespace veta

#endif // VETA_UTIL_RANDOM_UNIFORMRANDOMIZER_H_
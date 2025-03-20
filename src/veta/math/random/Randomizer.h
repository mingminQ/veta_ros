/**
 * --------------------------------------------------
 *
 * @file    Randomizer.h
 * @brief   Randomizer base utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_RANDOM_RANDOMIZER_H_
#define VETA_MATH_RANDOM_RANDOMIZER_H_

#include <memory>
#include <random>
#include <cassert>
#include <cstdint>
#include <algorithm>

namespace veta
{
    namespace random
    {
        /** @brief Base class of the randomizer */
        class Randomizer
        {
        // "Randomizer" member variables
        protected:

            // Local random seed
            std::uint_fast64_t m_localSeed;

            // Random number generator
            std::mt19937 m_generator;

        // "Randomizer" member function
        private:

            /** @brief Disable copy constructor */
            Randomizer(const Randomizer &) = delete;
            const Randomizer &operator=(const Randomizer &) = delete;

        protected:

            /** @brief Class constructor */
            Randomizer();

            /** @brief Class constructor with initial local seed */
            Randomizer(std::uint_fast64_t localSeed);

            /** @brief Class destructor */
            virtual ~Randomizer() = default;

        public:
            
            /** @brief Global random seed setter function */
            static void setGlobalSeed(std::uint_fast64_t globalSeed);

            /** @brief Global random seed getter function */
            static std::uint_fast64_t getGlobalSeed();

            /** @brief Local random seed setter and reset distribution function */
            virtual void setLocalSeed(std::uint_fast64_t localSeed);

            /** @brief Local random seed getter function */
            std::uint_fast64_t getLocalSeed();

            /** @brief Element shuffle function*/
            template <typename ClassIterator>
            void shuffle(ClassIterator begin, ClassIterator end)
            {
                std::shuffle(begin, end, m_generator);
            }

        }; // class Randomizer

    } // namespace random

} // namespace veta

#endif // VETA_MATH_RANDOM_RANDOMIZER_H_
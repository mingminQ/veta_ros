/**
 * --------------------------------------------------
 *
 * @file    Randomizer.cpp
 * @brief   Randomizer base utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/math/random/Randomizer.h"
#include "veta/util/LogHandler.h"

#include <chrono>
#include <mutex>
#include <limits>

namespace // anonymous
{
    class SeedGenerator
    {
    // "SeedGenerator" member variables
    private:

        // Initial seed
        bool m_isSeedGenerated {false};
        std::uint_fast64_t m_initialSeed;

        // Thread safety mutex locker
        std::mutex m_generatorMutex;

        // Random seed generator
        std::ranlux24_base m_generator;
        std::uniform_int_distribution<> m_uniformDist;

    // "SeedGenerator" member functions
    public:

        /** @brief Class constructor */
        SeedGenerator()
            : m_initialSeed(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch()).count())
            , m_generator(m_initialSeed)
            , m_uniformDist(1, std::numeric_limits<int>::max())
        {    
        }

        /** @brief Global random seed setter function */
        void setSeed(std::uint_fast64_t seed)
        {
            std::lock_guard<std::mutex> syncLock(m_generatorMutex);
            if(0 < seed)
            {
                if(m_isSeedGenerated)
                {
                    VETA_WARN("Random number generation already started, changing seed now will not lead to deterministic sampling");
                }
                else // seed is not generated
                {
                    m_initialSeed = seed;
                }
            }
            else // seed <= 0
            {
                if(m_isSeedGenerated)
                {
                    VETA_WARN("Random generator seed cannot be 0, seed has been igonored");
                    return;
                }
                VETA_WARN("Random generator seed cannot be 0, using 1 instead");
                seed = 1;
            }
            m_generator.seed(seed);
        }

        /** @brief Global random seed getter function */
        std::uint_fast64_t getInitialSeed()
        {
            std::lock_guard<std::mutex> syncLock(m_generatorMutex);
            return m_initialSeed;
        }

        /** @brief Global random seed getter function */
        std::uint_fast64_t getNextSeed()
        {
            std::lock_guard<std::mutex> syncLock(m_generatorMutex);
            m_isSeedGenerated = true;
            return m_uniformDist(m_generator);
        }

    }; // class SeedGenerator

    // Global random seed functions
    std::once_flag g_seedGeneratorFlag;
    std::unique_ptr<SeedGenerator> g_seedGenerator;

    /** @brief Seed generator initialization function */

    void initSeedGenerator()
    {
        g_seedGenerator.reset(new SeedGenerator());
    }

    /** @brief Seed generator instance getter function */
    SeedGenerator &getSeedGenerator()
    {
        std::call_once(g_seedGeneratorFlag, &initSeedGenerator);
        return *g_seedGenerator;
    }

} // namespace anonymous

// "veta::random::Randomizer" source

/** @brief Base class of the randomizer */
veta::random::Randomizer::Randomizer()
  : m_localSeed(getSeedGenerator().getNextSeed())
  , m_generator(m_localSeed)
{
    // @TODO add shpere data
}

/** @brief Class constructor */
veta::random::Randomizer::Randomizer(std::uint_fast64_t localSeed)
  : m_localSeed(localSeed)
  , m_generator(m_localSeed)
{
    // @TODO add shpere data
}

/** @brief Global random seed setter function */
void veta::random::Randomizer::setGlobalSeed(std::uint_fast64_t globalSeed)
{
    getSeedGenerator().setSeed(globalSeed);
}

/** @brief Global random seed getter function */
std::uint_fast64_t veta::random::Randomizer::getGlobalSeed()
{
    return getSeedGenerator().getInitialSeed();
}

/** @brief Local random seed setter and reset distribution function */
void veta::random::Randomizer::setLocalSeed(std::uint_fast64_t localSeed)
{
    m_localSeed = localSeed;
    m_generator.seed(m_localSeed);
}

/** @brief Local random seed getter function */
std::uint_fast64_t veta::random::Randomizer::getLocalSeed()
{
    return m_localSeed;
}
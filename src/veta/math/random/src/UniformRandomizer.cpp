/**
 * --------------------------------------------------
 *
 * @file    UniformRandomizer.cpp
 * @brief   Uniform randomizer utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/math/random/UniformRandomizer.h"

// "veta::random::UniformRandomizer" source

/** @brief Class constructor */
veta::random::UniformRandomizer::UniformRandomizer()
  : veta::random::Randomizer()
{
}

/** @brief Class constructor with initial local seed */
veta::random::UniformRandomizer::UniformRandomizer(std::uint_fast64_t localSeed) 
  : veta::random::Randomizer(localSeed)
{
}

/**
 * @brief Local random seed setter method and reset distribution
 * @param localSeed Local random seed
 */
void veta::random::UniformRandomizer::setLocalSeed(std::uint_fast64_t localSeed)
{
    veta::random::Randomizer::setLocalSeed(localSeed);
    m_uniformDist.reset();
}

/**
 * @brief Real random number generation method by uniform distribution
 * @param lowerBound Lower boundary of the result (default : 0.0)
 * @param upperBound Upper boundary of the result (default : 1.0)
 */
double veta::random::UniformRandomizer::uniformReal(double lowerBound, double upperBound)
{
    assert(lowerBound < upperBound);
    return (upperBound - lowerBound) * m_uniformDist(m_generator) + lowerBound;
}

/**
 * @brief Integer random number generation method by uniform distribution
 * @param lowerBound Lower boundary of the result
 * @param upperBound Upper boundary of the result
 */
int veta::random::UniformRandomizer::uniformInt(int lowerBound, int upperBound)
{
    auto result = (int)floor(uniformReal((double)lowerBound, (double)(upperBound) + 1.0));
    return (result > upperBound) ? upperBound : result;
}

/** @brief Boolean random number generation method by uniform distribution */
bool veta::random::UniformRandomizer::uniformBool()
{
    return m_uniformDist(m_generator) < 0.5;
}

// @TODO DataType uniformQuaternion()

// @TODO DataType uniformEulerRPY()

// @TODO add sphere data uniform randomization
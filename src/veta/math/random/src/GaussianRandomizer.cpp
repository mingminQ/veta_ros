/**
 * --------------------------------------------------
 *
 * @file    GaussianRandomizer.cpp
 * @brief   Gaussian randomizer utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/math/random/GaussianRandomizer.h"

// "veta::random::GaussianRandomizer" source

/** @brief Class constructor */
veta::random::GaussianRandomizer::GaussianRandomizer() 
  : veta::random::Randomizer()
{
}

/** @brief Class constructor with initial local seed */
veta::random::GaussianRandomizer::GaussianRandomizer(std::uint_fast64_t localSeed) 
  : veta::random::Randomizer(localSeed)
{
}

/**
 * @brief Local random seed setter method and reset distribution
 * @param localSeed Local random seed
 */
void veta::random::GaussianRandomizer::setLocalSeed(std::uint_fast64_t localSeed)
{
    veta::random::Randomizer::setLocalSeed(localSeed);
    m_normalDist.reset();
}

/**
 * @brief Real random number generation method by gaussian distribution
 * @param mean Mean of the gaussian distribution (default : 0.0)
 * @param stdDev Standard deviation of the gausian distribution (default : 1.0)
 */
double veta::random::GaussianRandomizer::gaussianReal(const double &mean, const double &stdDev)
{
    return m_normalDist(m_generator) * stdDev + mean;
}

/**
 * @brief Real random number generation method by folded gaussian distribution
 * @param lowerBound Lower boundary of the result
 * @param upperBound Upper boundary of the result
 * @param bias Foucusing value around upper boundary
 */
double veta::random::GaussianRandomizer::foldedGaussianReal(const double &lowerBound, const double &upperBound, const double &bias)
{
    assert(lowerBound < upperBound);
    double mean = upperBound - lowerBound;
    double half = gaussianReal(mean, mean / bias);

    half = (half > mean) ? (2.0 * mean - half) : half;
    double result = (half >= 0.0) ? (half + lowerBound) : lowerBound;
    
    return (result > upperBound) ? upperBound : result;
}

/**
 * @brief Integer random number generation method by folded gaussian distribution
 * @param lowerBound Lower boundary of the result
 * @param upperBound Upper boundary of the result
 * @param bias Foucusing value around upper boundary
 */
int veta::random::GaussianRandomizer::foldedGaussianInt(const int &lowerBound, const int &upperBound, const double &bias)
{
    auto result = (int)floor(foldedGaussianReal((double)lowerBound, (double)(upperBound) + 1.0, bias));
    return (result > upperBound) ? upperBound : result;
}
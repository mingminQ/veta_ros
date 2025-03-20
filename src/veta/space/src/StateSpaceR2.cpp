/**
 * --------------------------------------------------
 *
 * @file    StateSpaceR2.cpp
 * @brief   2D real vector state space source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/space/StateSpaceR2.h"

#include <iostream>
#include <cstring>
#include <limits>
#include <cmath>

// "veta::space::StateSpaceR2" source

/** @brief State memory allocation function */
veta::space::State *veta::space::StateSpaceR2::newState() const
{
    auto *state = new StateR2();
    state->m_elements = new double[StateR2::ELEMENT_NUM];
    return state;
}

/** @brief State memory allocation function */
void veta::space::StateSpaceR2::deleteState(State *state) const
{
    auto *stateR2 = static_cast<StateR2 *>(state);
    delete[] stateR2->m_elements;
    delete stateR2;
}

/** @brief State copy function */
void veta::space::StateSpaceR2::copyState(State *dstState, const State *srcState) const
{
    const auto *src = static_cast<const StateR2 *>(srcState);
    auto *dst = static_cast<StateR2 *>(dstState);
    std::memcpy(dst->m_elements, src->m_elements, StateR2::ELEMENT_NUM * sizeof(double));
}

/** @brief Print state elements */
void veta::space::StateSpaceR2::printState(const State *state) const
{
    const auto *stateR2 = static_cast<const StateR2 *>(state);
    std::cout << "StateR2: [" 
              << stateR2->m_elements[StateR2::X] << ", "
              << stateR2->m_elements[StateR2::Y] 
              << "]" << std::endl;
    std::cout << "---" <<std::endl;
}

/** @brief Distance between two states */
double veta::space::StateSpaceR2::distance(const State *state1, const State *state2) const
{
    const auto *s1 = static_cast<const StateR2 *>(state1);
    const auto *s2 = static_cast<const StateR2 *>(state2);
    
    double dist = 0.0;
    for(int idx = 0; idx < StateR2::ELEMENT_NUM; idx++)
    {
        double delta = s2->m_elements[idx] - s1->m_elements[idx];
        dist += (delta * delta);
    }
    return std::sqrt(dist);
}

/** @brief Interpolate between two states */
veta::space::State *veta::space::StateSpaceR2::interpolate(const State *state1, const State *state2, const double &rate) const
{
    const auto *s1 = static_cast<const StateR2 *>(state1);
    const auto *s2 = static_cast<const StateR2 *>(state2);

    State *result = newState();
    auto *stateR2 = static_cast<StateR2 *>(result);

    for(int idx = 0; idx < StateR2::ELEMENT_NUM; idx++)
    {
        stateR2->m_elements[idx] = s1->m_elements[idx] + rate * (s2->m_elements[idx] - s1->m_elements[idx]);
    }
    return stateR2;
}

/** @brief Reduce state to state bounds */
void veta::space::StateSpaceR2::enforceStateBounds(State *state) const
{
    auto *stateR2 = static_cast<StateR2 *>(state);
    for(int idx = 0; idx < StateR2::ELEMENT_NUM; idx++)
    {
        if(stateR2->m_elements[idx] < m_lowerBound[idx])
        {
            stateR2->m_elements[idx] = m_lowerBound[idx];
        }
        else if(m_upperBound[idx] < stateR2->m_elements[idx])
        {
            stateR2->m_elements[idx] = m_upperBound[idx];
        }
    }
}

/** @brief Check statisfaction of the state bound */
bool veta::space::StateSpaceR2::checkStateBounds(const State *state) const
{
    const auto *stateR2 = static_cast<const StateR2 *>(state);
    for(int idx = 0; idx < StateR2::ELEMENT_NUM; idx++)
    {
        if(stateR2->m_elements[idx] < m_lowerBound[idx] || m_upperBound[idx] < stateR2->m_elements[idx])
        {
            return false;
        }
    }
    return true;
}

/** @brief Check state equality */
bool veta::space::StateSpaceR2::checkEqualStates(const State *state1, const State *state2) const
{
    const auto *s1 = static_cast<const StateR2 *>(state1);
    const auto *s2 = static_cast<const StateR2 *>(state2);
    
    double dist = 0.0;
    for(int idx = 0; idx < StateR2::ELEMENT_NUM; idx++)
    {
        double delta = std::fabs(s2->m_elements[idx] - s1->m_elements[idx]);
        if((2.0 * std::numeric_limits<double>::epsilon()) < delta)
        {
            return false;
        }
    }
    return true;
}
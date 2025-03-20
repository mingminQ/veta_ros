/**
 * --------------------------------------------------
 *
 * @file    StateSpaceSO2.cpp
 * @brief   2D rotation state space source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */
#include "veta/space/StateSpaceSO2.h"

#include <iostream>
#include <cstring>
#include <limits>
#include <cmath>

// "veta::space::StateSpaceSO2" source

/** @brief State memory allocation function */
veta::space::State *veta::space::StateSpaceSO2::newState() const
{
    auto *state = new StateSO2();
    return state;
}

/** @brief State memory allocation function */
void veta::space::StateSpaceSO2::deleteState(State *state) const
{
    auto *stateSO2 = static_cast<StateSO2 *>(state);
    delete stateSO2;
}

/** @brief State copy function */
void veta::space::StateSpaceSO2::copyState(State *dstState, const State *srcState) const
{
    dstState->state_cast<StateSO2>()->m_element = srcState->state_cast<StateSO2>()->m_element;
}

/** @brief Print state elements */
void veta::space::StateSpaceSO2::printState(const State *state) const
{
    const auto *stateSO2 = static_cast<const StateSO2 *>(state);
    std::cout << "StateSO2: [" 
              << stateSO2->m_element 
              << "]" << std::endl;
    std::cout << "---" <<std::endl;
}

/** @brief Distance between two states */
double veta::space::StateSpaceSO2::distance(const State *state1, const State *state2) const
{
    if((!checkStateBounds(state1)) || (!checkStateBounds(state2)))
    {
        VETA_WARN("StateSO2 elements do not satisfy bounds, call StateSpaceSO2::enforceStateBounds()");
        return 0.0;
    }

    double e1 = static_cast<const StateSO2 *>(state1)->m_element;
    double e2 = static_cast<const StateSO2 *>(state2)->m_element;

    double dist = std::fabs(e2 - e1);
    return (M_PI < dist) ? (2.0 * M_PI - dist) : dist;
}

/** @brief Interpolate between two states */
veta::space::State *veta::space::StateSpaceSO2::interpolate(const State *state1, const State *state2, const double &rate) const
{
    const double e1 = static_cast<const StateSO2 *>(state1)->m_element;
    const double e2 = static_cast<const StateSO2 *>(state2)->m_element;

    State *state = newState();
    auto *stateSO2 = static_cast<StateSO2 *>(state);

    double delta = e2 - e1;
    if(std::fabs(delta) <= M_PI)
    {
        stateSO2->m_element = e1 + rate * delta;
    }
    else
    {
        if(0.0 < delta)
        {
            delta = 2.0 * M_PI - delta;
        }
        else
        {
            delta = -2.0 * M_PI - delta;
        }

        double value = e1 - rate * delta;
        if(M_PI < value)
        {
            stateSO2->m_element = value - 2.0 * M_PI;
        }
        else if(value < -M_PI)
        {
            stateSO2->m_element = value + 2.0 * M_PI;
        }
    }
    return stateSO2;
}

/** @brief Reduce state to state bounds */
void veta::space::StateSpaceSO2::enforceStateBounds(State *state) const
{
    auto *stateSO2 = static_cast<StateSO2 *>(state);
    double value = std::fmod(stateSO2->m_element, 2.0 * M_PI);

    if(value < -M_PI)
    {
        value += (2.0 * M_PI);
    }
    else if(M_PI <= value)
    {
        value -= (2.0 * M_PI);
    }
    stateSO2->m_element = value;
}

/** @brief Check statisfaction of the state bound */
bool veta::space::StateSpaceSO2::checkStateBounds(const State *state) const
{
    const double e = static_cast<const StateSO2 *>(state)->m_element;
    if(-M_PI <= e && e < M_PI)
    {
        return true;
    }
    return false;
}

/** @brief Check state equality */
bool veta::space::StateSpaceSO2::checkEqualStates(const State *state1, const State *state2) const
{
    const double e1 = static_cast<const StateSO2 *>(state1)->m_element;
    const double e2 = static_cast<const StateSO2 *>(state2)->m_element;

    double delta = std::fabs(e2 - e1);
    if(delta < (2.0 * std::numeric_limits<double>::epsilon()))
    {
        return true;
    }
    return false;
}
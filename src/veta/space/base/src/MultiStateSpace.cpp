/**
 * --------------------------------------------------
 *
 * @file    MultiStateSpace.cpp
 * @brief   Multiple state space base class source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/space/base/MultiStateSpace.h"

#include <iostream>
#include <cstdio>

// "veta::space::MultiState" source

/** @brief State component access operator */
const veta::space::State *veta::space::MultiState::operator[](const int &idx) const
{
    return m_states[idx];
}

/** @brief State component access operator */
veta::space::State *veta::space::MultiState::operator[](const int &idx)
{
    return m_states[idx];
}

// "veta::space::MultiStateSpace" source

/** @brief Add component state space */
void veta::space::MultiStateSpace::addComponentSpace(const StateSpacePtr &stateSpace, const double &weight)
{
    m_spaces.push_back(stateSpace);
    m_weight.push_back(weight);
}

/** @brief State memory allocation function */
veta::space::State *veta::space::MultiStateSpace::newState() const
{
    auto *state = new MultiState();
    state->m_states = new State *[m_spaces.size()];

    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        state->m_states[idx] = m_spaces[idx]->newState();
    }
    return static_cast<State *>(state);
}

/** @brief State memory allocation function */
void veta::space::MultiStateSpace::deleteState(State *state) const
{
    auto *multiState = static_cast<MultiState *>(state);
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        m_spaces[idx]->deleteState(multiState->m_states[idx]);
    }
    delete[] multiState->m_states;
    delete multiState;
}

/** @brief State copy function */
void veta::space::MultiStateSpace::copyState(State *dstState, const State *srcState) const
{
    const auto *src = static_cast<const MultiState *>(srcState);
    auto *dst = static_cast<MultiState *>(dstState);
    
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        m_spaces[idx]->copyState(dst->m_states[idx], src->m_states[idx]);
    }
}

/** @brief Print state elements */
void veta::space::MultiStateSpace::printState(const State *state) const
{
}

/** @brief Distance between two states */
double veta::space::MultiStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *s1 = static_cast<const MultiState *>(state1);
    const auto *s2 = static_cast<const MultiState *>(state2);

    double dist = 0.0;
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        dist += m_weight[idx] * m_spaces[idx]->distance(s1->m_states[idx], s2->m_states[idx]);
    }
    return dist;
}

/** @brief Interpolate between two states */
veta::space::State *veta::space::MultiStateSpace::interpolate(const State *state1, const State *state2, const double &rate) const
{
    const auto *s1 = static_cast<const MultiState *>(state1);
    const auto *s2 = static_cast<const MultiState *>(state2);

    auto *s3 = new MultiState();
    s3->m_states = new State *[m_spaces.size()];
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        s3->m_states[idx] = m_spaces[idx]->interpolate(s1->m_states[idx], s2->m_states[idx], rate);
    }
    return static_cast<State *>(s3);
}

/** @brief Reduce state to state bounds */
void veta::space::MultiStateSpace::enforceStateBounds(State *state) const
{
    auto *s = static_cast<MultiState *>(state);
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        m_spaces[idx]->enforceStateBounds(s->m_states[idx]);
    }
}

/** @brief Check statisfaction of the state bound */
bool veta::space::MultiStateSpace::checkStateBounds(const State *state) const
{
    const auto *s = static_cast<const MultiState *>(state);
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        if(!m_spaces[idx]->checkStateBounds(s->m_states[idx]))
        {
            return false;
        }
    }
    return true;
}

/** @brief Check state equality */
bool veta::space::MultiStateSpace::checkEqualStates(const State *state1, const State *state2) const
{
    const auto *s1 = static_cast<const MultiState *>(state1);
    const auto *s2 = static_cast<const MultiState *>(state2);

    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        if(!m_spaces[idx]->checkEqualStates(s1->m_states[idx], s2->m_states[idx]))
        {
            return false;
        }
    }
    return true;
}
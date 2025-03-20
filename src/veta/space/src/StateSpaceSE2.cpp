/**
 * --------------------------------------------------
 *
 * @file    StateSpaceSE2.cpp
 * @brief   2D position rotation state space source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/space/StateSpaceSE2.h"

#include <iostream>
#include <memory>

// "veta::space::StateSpaceSE2" source

/** @brief Class constructor */
veta::space::StateSpaceSE2::StateSpaceSE2()
{
    addComponentSpace(std::make_shared<StateSpaceR2>(), 1.0);
    addComponentSpace(std::make_shared<StateSpaceSO2>(), 0.5);
}

/** @brief Class destructor */
veta::space::StateSpaceSE2::~StateSpaceSE2()
{
    for(int idx = 0; idx < m_spaces.size(); idx++)
    {
        m_spaces[idx].reset();
    }
}

/** @brief Print state elements */
void veta::space::StateSpaceSE2::printState(const State *state) const
{
    const auto *stateSE2 = static_cast<const StateSE2 *>(state);
    std::cout << "StateSE2: ["
              << stateSE2->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::X] << ", "
              << stateSE2->state_cast<StateR2>(StateSE2::POSITION)->m_elements[StateR2::Y] << ", "
              << stateSE2->state_cast<StateSO2>(StateSE2::ROTATION)->m_element
              << "]" << std::endl;
    std::cout << "---" << std::endl;
}
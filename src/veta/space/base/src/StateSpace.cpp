/**
 * --------------------------------------------------
 *
 * @file    StateSpace.cpp
 * @brief   State space base class source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/space/base/StateSpace.h"

// "veta::space::StateSpace" source

/** @brief State copy function */
veta::space::State *veta::space::StateSpace::cloneState(const State *srcState) const
{
    State *dstState = newState();
    copyState(dstState, srcState);
    return dstState;
}
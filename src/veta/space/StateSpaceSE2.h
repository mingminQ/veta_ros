/**
 * --------------------------------------------------
 *
 * @file    StateSpaceSE2.h
 * @brief   2D position rotation state space
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_SPACE_STATESPACESE2_H_
#define VETA_SPACE_STATESPACESE2_H_

#include "veta/space/base/MultiStateSpace.h"
#include "veta/space/StateSpaceR2.h"
#include "veta/space/StateSpaceSO2.h"

namespace veta
{
    namespace space
    {
        /** @brief 2D position and rotation state */
        class StateSE2 : public MultiState
        {
        public:

            // "StateSE2" components
            enum Components
            {
                POSITION = 0,
                ROTATION = 1,
                COMPONENT_NUM
            
            }; // enum Components 

            // "StateSE2" Elements
            enum Elements
            {
                X   = 0,
                Y   = 1,
                YAW = 2,
                ELEMENT_NUM

            }; // enum Elements

        }; // class StateSE2

        /** @brief 2D position and rotation state space */
        class StateSpaceSE2 : public MultiStateSpace
        {
        // "StateSpaceSE2" memeber functions
        public:

            /** @brief Class constructor */
            StateSpaceSE2();

            /** @brief Class destructor */
            ~StateSpaceSE2() override;

            /** @brief Print state elements */
            void printState(const State *state) const override;

        }; // class StateSpaceSE2

    } // namespace space

} // namespace veta

#endif // VETA_SPACE_STATESPACESE2_H_
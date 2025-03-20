/**
 * --------------------------------------------------
 *
 * @file    StateSpaceSO2.h
 * @brief   2D rotation state space
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_SPACE_STATESPACESO2_H_
#define VETA_SPACE_STATESPACESO2_H_

#include "veta/space/base/StateSpace.h"

namespace veta
{
    namespace space
    {
        /** @brief 2D rotation state */
        class StateSO2 : public State
        {
        // "StateSO2" member variables
        public:

            enum Elements
            {
                THETA = 1,
                ELEMENT_NUM

            }; // enum Elements

            double m_element;

        }; // class StateSO2

        /** @brief 2D rotation state space */
        class StateSpaceSO2 : public StateSpace
        {
        // "StateSpaceR2" member variables
        public:

            // Default state type
            using DefaultState = StateSO2;

        // "StateSpaceR2" member functions
        public:

            /** @brief State memory allocation function */
            State *newState() const override;

            /** @brief State memory allocation function */
            void deleteState(State *state) const override;

            /** @brief State copy function */
            void copyState(State *dstState, const State *srcState) const override;

            /** @brief Print state elements */
            void printState(const State *state) const override;

            /** @brief Distance between two states */
            double distance(const State *state1, const State *state2) const override;

            /** @brief Interpolate between two states */
            State *interpolate(const State *state1, const State *state2, const double &rate) const override;

            /** @brief Reduce state to state bounds */
            void enforceStateBounds(State *state) const override;

            /** @brief Check statisfaction of the state bound */
            bool checkStateBounds(const State *state) const override;

            /** @brief Check state equality */
            bool checkEqualStates(const State *state1, const State *state2) const override;

        }; // class StateSpaceSO2

    } // namespace space

} // namespace veta

#endif // VETA_SPACE_STATESPACESO2_H_
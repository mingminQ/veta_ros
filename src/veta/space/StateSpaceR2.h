/**
 * --------------------------------------------------
 *
 * @file    StateSpaceR2.h
 * @brief   2D real vector state space
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_SPACE_STATESPACER2_H_
#define VETA_SPACE_STATESPACER2_H_

#include "veta/util/generic/GenericRange.h"
#include "veta/space/base/StateSpace.h"

namespace veta
{
    namespace space
    {
        /** @brief 2D real vector state */
        class StateR2 : public State
        {
        // "StateR2" member variables
        public:

            // "StateR2" elements
            enum Elements
            {
                X = 0,
                Y = 1,
                ELEMENT_NUM

            }; // enum Elements

            // "StateR2" elements
            double *m_elements;

        }; // class StateR2

        /** @brief 2D real vector state space */
        class StateSpaceR2 : public StateSpace
        {
        // "StateSpaceR2" member variables
        public:

            // Default state type
            using DefaultState = StateR2;

        private:

            // State boundary
            std::vector<double> m_lowerBound;
            std::vector<double> m_upperBound;

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

        }; // class StateSpaceR2

    } // namespace space

} // namespace veta

#endif // VETA_SPACE_STATESPACER2_H_
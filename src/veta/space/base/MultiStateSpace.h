/**
 * --------------------------------------------------
 *
 * @file    MultiStateSpace.h
 * @brief   Multiple state space base class
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_SPACE_BASE_MULTISTATESPACE_H_
#define VETA_SPACE_BASE_MULTISTATESPACE_H_

#include "veta/space/base/StateSpace.h"

#include <vector>

namespace veta
{
    namespace space
    {
        /** @brief Multiple state interface class */
        class MultiState : public State
        {
        // "MultiState" member variables
        public:

            // Component states
            State **m_states;

        // "MultiState" member functions
        public:

            /** @brief State const data type casting function */
            template <typename StateType>
            const StateType *state_cast(const int &idx) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<StateType *, State *>));
                return static_cast<const StateType *>(m_states[idx]);
            }

            /** @brief State data type casting function */
            template <typename StateType>
            StateType *state_cast(const int &idx)
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<StateType *, State *>));
                return static_cast<StateType *>(m_states[idx]);
            }

            /** @brief State component access operator */
            const State *operator[](const int &idx) const;

            /** @brief State component access operator */
            State *operator[](const int &idx);

        }; // class MultiState

        /** @brief Multiple state space interface class */
        class MultiStateSpace : public StateSpace
        {
        // "MultiStateSpace" member variables
        public:

            // Default state type
            using DefaultState = MultiState;

        protected:

            // "MultiStateSpace" component space
            std::vector<StateSpacePtr> m_spaces;

            // Weoght of the each state spaces
            std::vector<double> m_weight;

        // "MultiStateSpace" member functions
        protected:

            /** @brief Add component state space */
            void addComponentSpace(const StateSpacePtr &stateSpace, const double &weight);

        public:

            /** @brief State space const data type casting function */
            template <typename SpaceType>
            const SpaceType *state_cast(const int &idx) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<SpaceType *, StateSpace *>));
                return static_cast<const SpaceType *>(m_spaces[idx]);
            }

            /** @brief State space data type casting function */
            template <typename SpaceType>
            SpaceType *state_cast(const int &idx)
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<SpaceType *, StateSpace *>));
                return static_cast<SpaceType *>(m_spaces[idx]);
            }

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
            
        }; // class MultiStateSpace

    } // namespace veta

} // namespace veta

#endif // VETA_SPACE_BASE_MULTISTATESPACE_H_
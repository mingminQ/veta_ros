/**
 * --------------------------------------------------
 *
 * @file    StateSpace.h
 * @brief   State space base class
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_SPACE_BASE_STATESPACE_H_
#define VETA_SPACE_BASE_STATESPACE_H_

#include "veta/util/generic/GenericParameter.h"
#include "veta/util/PointerForward.h"

#include "boost/concept_check.hpp"

namespace veta
{
    namespace space
    {
        /** @brief Base class of the state */
        class State
        {
        // "State" member functions
        private:

            /** @brief State copy constructor is disabled */
            State(const State &) = delete;

            /** @brief State copy operator is disabled */
            State &operator=(const State &) = delete;

        protected:

            /** @brief Class constructor */
            State() = default;

            /** @brief Class destructor */
            ~State() = default;

        public:

            /** @brief State const data type casting function */
            template <typename StateType>
            const StateType *state_cast() const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<StateType *, State *>));
                return static_cast<const StateType *>(this);
            }

            /** @brief State data type casting function */
            template <typename StateType>
            StateType *state_cast()
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<StateType *, State *>));
                return static_cast<StateType *>(this);
            }

        }; // class State

        // Alias of the state space shared_ptr
        CLASS_POINTER_FORWARD(StateSpace);

        /** @brief Base class of the state space */
        class StateSpace
        {
        // "StateSpace" member variables
        public:

            // Default state type
            using DefaultState = State;

        // "StateSpace" member functions
        private:

            /** @brief State space copy constructor is disabled */
            StateSpace(const StateSpace &) = delete;

            /** @brief State space copy operator is disabled */
            StateSpace &operator=(const StateSpace &) = delete;

        public:

            /** @brief Class constructor */
            StateSpace() = default;

            /** @brief Class destructor */
            virtual ~StateSpace() = default;

            /** @brief State space const data type casting function */
            template <typename SpaceType>
            const SpaceType *state_cast() const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<SpaceType *, StateSpace *>));
                return static_cast<const SpaceType *>(this);
            }

            /** @brief State space data type casting function */
            template <typename SpaceType>
            SpaceType *state_cast()
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<SpaceType *, StateSpace *>));
                return static_cast<SpaceType *>(this);
            }

            /** @brief State memory allocation function */
            virtual State *newState() const = 0;

            /** @brief State memory allocation function */
            virtual void deleteState(State *state) const = 0;

            /** @brief State copy function */
            virtual void copyState(State *dstState, const State *srcState) const = 0;

            /** @brief State copy function */
            State *cloneState(const State *state) const;

            /** @brief Print state elements */
            virtual void printState(const State *state) const = 0;

            /** @brief Distance between two states */
            virtual double distance(const State *state1, const State *state2) const = 0;

            /** @brief Interpolate between two states */
            virtual State *interpolate(const State *state1, const State *state2, const double &rate) const = 0;

            /** @brief Reduce state to state bounds */
            virtual void enforceStateBounds(State *state) const = 0;

            /** @brief Check statisfaction of the state bound */
            virtual bool checkStateBounds(const State *state) const = 0;

            /** @brief Check state equality */
            virtual bool checkEqualStates(const State *state1, const State *state2) const = 0;

            // @TODO State sampler

        }; // class StateSpace

    } // namespace space

} // namespace veta

#endif // VETA_SPACE_BASE_STATESPACE_H_
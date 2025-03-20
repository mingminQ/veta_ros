/**
 * --------------------------------------------------
 *
 * @file    GenericRange.h
 * @brief   Generic range utility
 * @author  Minkyu Kil
 * @date    2025-02-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_GENERIC_GENERICRANGE_H_
#define VETA_UTIL_GENERIC_GENERICRANGE_H_

#include "boost/concept_check.hpp"
#include <functional>
#include <algorithm>
#include <limits>
#include <vector>

namespace veta
{
    // Infinity value of the double
    #define INF_DOUBLE std::numeric_limits<double>::infinity();

    /** @brief Base class of the generic range */
    class GenericRange
    {
    // "GenericRange" member variables
    public:

        // Range type of the generic range
        enum RangeType
        {
            BOUND    = 0,
            LIST     = 1,
            FUNCTION = 2

        }; // enum RangeType

    private:

        // Derived generic range type
        int m_rangeType;

    // "GenericRange" member functions
    private:

        /** @brief Copy constructor is disabled */
        GenericRange(const GenericRange &) = delete;

        /** @brief Copy operator is disabled */
        GenericRange &operator=(const GenericRange &) = delete;

    public:

        /** @brief Class constructor */
        GenericRange(const int &rangeType);

        /** @brief Class destructor */
        virtual ~GenericRange() = default;

        /** @brief Generic range const data type casting function */
        template <typename RangeType>
        const RangeType *range_cast() const
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<RangeType *, GenericRange *>));
            return static_cast<const RangeType *>(this);
        }

        /** @brief Generic range data type casting function */
        template <typename RangeType>
        RangeType *range_cast()
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<RangeType *, GenericRange *>));
            return static_cast<RangeType *>(this);
        }

        /** @brief Return derived generic parameter type */
        int getRangeType() const;

    }; // class GenericRange

    /** @brief Boundary range class */
    template <typename T>
    class GenericRangeBound : public GenericRange
    {
    // "GenericRangeBound" member variables
    private:

        // Range boundaries
        T m_lowerBound;
        T m_upperBound;

    // "GenericRangeBound" member functions
    public:

        /** @brief Class constructor */
        GenericRangeBound(const T &lowerBound, const T &upperBound)
        : GenericRange(BOUND), m_lowerBound(lowerBound), m_upperBound(upperBound)
        {
        }
        
        /** @brief Input value validation check function */
        bool checkValidation(const T &value)
        {
            return (m_lowerBound <= value) && (value <= m_upperBound);
        }

    }; // class GenericRangeBound

    /** @brief Available value list range class */
    template <typename T>
    class GenericRangeList : public GenericRange
    {
    // "GenericRangeList" member variables
    private:

        // Available value list
        std::vector<T> m_list;

    // "GenericRangeList" member functions
    public:

        /** @brief Class constructor */
        GenericRangeList(const std::vector<T> &list)
        : GenericRange(LIST), m_list(list)
        {
        }
        
        /** @brief Input value validation check function */
        bool checkValidation(const T &value)
        {
            auto iter = std::find(m_list.begin(), m_list.end(), value);
            return m_list.end() != iter;
        }

    }; // class GenericRangeList

    /** @brief Validation function range class */
    template <typename T>
    class GenericRangeFn : public GenericRange
    {
    // "GenericRangeFn" member variables
    private:

        // Validation function
        std::function<bool(const T &)> m_fn;

    // "GenericRangeFn" member functions
    public:

        /** @brief Class constructor */
        GenericRangeFn(std::function<bool(const T &)> fn) 
          : GenericRange(FUNCTION), m_fn(fn)
        {
        }

        /** @brief Input value validation check function */
        bool checkValidation(const T &value)
        {
            return m_fn(value);
        }

    }; // class GenericRangeFn
}

#endif // VETA_UTIL_GENERIC_GENERICRANGE_H_
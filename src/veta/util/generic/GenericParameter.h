/**
 * --------------------------------------------------
 *
 * @file    GenericParameter.h
 * @brief   Generic parameter utility
 * @author  Minkyu Kil
 * @date    2025-02-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_GENERIC_GENERICPARAMETER_H_
#define VETA_UTIL_GENERIC_GENERICPARAMETER_H_

#include "veta/util/generic/GenericRange.h"
#include "veta/util/LogHandler.h"
#include "veta/util/Exception.h"

#include <unordered_map>
#include <string>
 
namespace veta
{
    /** @brief Base class of the parameter */
    class Parameter
    {
    // "Parameter" member functions
    private:

        /** @brief Copy constructor is disabled */
        Parameter(const Parameter &) = delete;

        /** @brief Copy operator is disabled */
        Parameter &operator=(const Parameter &) = delete;

    public:

        /** @brief Class constructor */
        Parameter() = default;

        /** @brief Class destructor */
        virtual ~Parameter() = default;

        /** @brief Parameter const data type casting function */
        template <typename ParamType>
        const ParamType *param_cast() const
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<ParamType *, Parameter *>));
            return static_cast<const ParamType *>(this);
        }

        /** @brief Parameter data type casting function */
        template <typename ParamType>
        ParamType *param_cast()
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<ParamType *, Parameter *>));
            return static_cast<ParamType *>(this);
        }

    }; // class Parameter

    /** @brief Type specified parameter class */
    template <typename T>
    class ParameterTyped : public Parameter
    {
    // "ParameterTyped" member variables
    private:

        // Parameter name
        std::string m_name;

        // Parameter range
        GenericRange *m_range;

        // Parameter value
        T m_value;

    // "ParameterTyped" member functions
    public:

        /** @brief Class constructor */
        ParameterTyped(const std::string &name, const T &defaultValue, GenericRange *range)
          : m_name(name), m_range(range)
        {
            set(defaultValue);
        }

        /** @brief Class destructor */
        ~ParameterTyped() override
        {
            delete m_range;
        }

        /** @brief Parameter value setter function */
        bool set(const T &value)
        {
            bool isValid = false;
            switch (m_range->getRangeType())
            {
            case GenericRange::BOUND: // GenericRangeBound
                isValid = m_range->range_cast<GenericRangeBound<T>>()->checkValidation(value);
                break;
            case GenericRange::LIST: // GenericRangeList
                isValid = m_range->range_cast<GenericRangeList<T>>()->checkValidation(value);
                break;
            case GenericRange::FUNCTION: // GenericRangeFn
                isValid = m_range->range_cast<GenericRangeFn<T>>()->checkValidation(value);
                break;
            }
            if(!isValid)
            {
                VETA_WARN("Input of the parameter [%s] is out of the range, input is ignored", m_name.c_str());
                return false;
            }
            m_value = value;
            return true;
        }

        /** @brief Parameter value getter function */
        T get() const
        {
            return m_value;
        }

    }; // class ParameterTyped

    /** @brief Parameter management container class */
    class GenericParameter
    {
    // "GenericParameter" member variables
    private:
    
        // Parameter container
        using Key = int;
        std::unordered_map<Key, Parameter *> m_parameters;

    // "GenericParameter" member functions
    private:

        /** @brief Find parameter pointer with key */
        template <typename T>
        ParameterTyped<T> *findParameter(const Key &key) const
        {
            // Find parameters with key
            auto iter = m_parameters.find(key);

            // Non-declared parameter case
            if(iter == m_parameters.end())
            {
                VETA_WARN("Non-declare parameter key, cannot access to parameter");
                return nullptr;
            }

            // Parameter type inference error
            auto *parameterPtr = iter->second->param_cast<ParameterTyped<T>>();
            if(parameterPtr == nullptr)
            {
                VETA_WARN("Data type of the parameter mismatch, cannot access to parameter");
                return nullptr;
            }
            return parameterPtr;
        }

    public:

        /** @brief Class constructor */
        GenericParameter() = default;

        /** @brief Class destructor */
        ~GenericParameter();

        /** 
         * @brief Parameter declaration function
         * @details Generic range type is boundary
         */
        template <typename T>
        void declare(const Key &key, const std::string &name, const T &value, const T &lowerBound, const T &upperBound)
        {
            GenericRange *range = new GenericRangeBound<T>(lowerBound, upperBound);
            m_parameters[key] = new ParameterTyped<T>(name, value, range);
        }

        /** 
         * @brief Parameter declaration function
         * @details Generic range type is available value list
         */
        template <typename T>
        void declare(const Key &key, const std::string &name, const T &value, const std::vector<T> &list)
        {
            GenericRange *range = new GenericRangeList<T>(list);
            m_parameters[key] = new ParameterTyped<T>(name, value, range);
        }

        /** 
         * @brief Parameter declaration function
         * @details Generic range type is value validation function
         */
        template <typename T>
        void declare(const Key &key, const std::string &name, const T &value, std::function<bool(const T &)> fn)
        {
            GenericRange *range = new GenericRangeFn<T>(fn);
            m_parameters[key] = new ParameterTyped<T>(name, value, range);
        }

        /** @brief Multiple parameter value setter function */
        template <typename T1, typename T2, typename... Args>
        bool set(const Key &key1, const T1 &value1, const Key &key2, const T2 &value2, Args... args)
        {
            bool isSet = set(key1, value1);
            return isSet && set(key2, value2, args...);
        }

        /** @brief Parameter value setter function */
        template <typename T>
        bool set(const Key &key, const T &value)
        {
            ParameterTyped<T> * parameterPtr = findParameter<T>(key);
            if(parameterPtr == nullptr)
            {
                return false;
            }
            parameterPtr->set(value);
            return true;
        }

        /** @brief Recursion parameter value setter function exit condition */
        bool set() const;

        /** @brief Parameter value getter function */
        template <typename T>
        T get(const Key &key) const
        {
            ParameterTyped<T> * parameterPtr = findParameter<T>(key);
            if(parameterPtr == nullptr)
            {
                Exception("Generic parameter pointer casting error");
            }
            return parameterPtr->get();
        }

    }; // class GenericParameter

} // namespace veta

#endif // VETA_UTIL_GENERIC_GENERICPARAMETER_H_
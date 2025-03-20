/**
 * --------------------------------------------------
 *
 * @file    Exception.h
 * @brief   Exception definition
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_EXCEPTION_H_
#define VETA_UTIL_EXCEPTION_H_

#include <string>
#include <stdexcept>

namespace veta
{
    /** @brief Exception class inhereted STL runtime_error */
    class Exception : public std::runtime_error
    {
    // "Exeption" member functions
    public:

        /** @brief Class construcor requires error type */
        explicit Exception(const std::string &errorType) : std::runtime_error(errorType)
        {
        }

        /** @brief Class construcor requires error type and additional message */
        Exception(const std::string &errorType, const std::string &message) : std::runtime_error(errorType + " " + message)
        {
        }

        /** @brief Class destrucor requires no exception */
        ~Exception() noexcept override = default;

    }; // class Exception

} // namespace veta

#endif // VETA_UTIL_EXCEPTION_H_
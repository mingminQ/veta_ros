/**
 * --------------------------------------------------
 *
 * @file    GenericParameter.cpp
 * @brief   Generic parameter utility source
 * @author  Minkyu Kil
 * @date    2025-02-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/util/generic/GenericParameter.h"

// "veta::GenericParameter" source

/** @brief Class destructor */
veta::GenericParameter::~GenericParameter()
{
    for(const auto &parameter : m_parameters)
    {
        delete parameter.second;
    }
}

/** @brief Recursion parameter value setter function exit condition */
bool veta::GenericParameter::set() const
{
    return true;
}
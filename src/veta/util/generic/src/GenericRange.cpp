/**
 * --------------------------------------------------
 *
 * @file    GenericRange.cpp
 * @brief   Generic range utility source
 * @author  Minkyu Kil
 * @date    2025-02-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/util/generic/GenericRange.h"

// "veta::GenericRange" source

/** @brief Class constructor */
veta::GenericRange::GenericRange(const int &rangeType) : m_rangeType(rangeType)
{
}

/** @brief Return derived generic parameter type */
int veta::GenericRange::getRangeType() const
{
    return m_rangeType;
}
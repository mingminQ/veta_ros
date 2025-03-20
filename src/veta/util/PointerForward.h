/**
 * --------------------------------------------------
 *
 * @file    PointerForward.h
 * @brief   Pointer forward declaration macro
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_POINTERFORWARD_H_
#define VETA_UTIL_POINTERFORWARD_H_

#include <memory>

/** @brief Alias of the class shared_ptr */
#define CLASS_POINTER_FORWARD(CLASS_NAME)               \
    class CLASS_NAME;                                   \
    using CLASS_NAME##Ptr = std::shared_ptr<CLASS_NAME>

#endif // VETA_UTIL_POINTERFORWARD_H_
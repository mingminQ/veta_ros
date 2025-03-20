/**
 * --------------------------------------------------
 *
 * @file    Timer.h
 * @brief   ROS Noetic timer macro
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_NOETIC_UTIL_TIMER_H_
#define VETA_NOETIC_UTIL_TIMER_H_

#include "veta/util/LogHandler.h"
#include "ros/ros.h"

/**
 * @brief Timer start macro
 * @details Using ROS time for timer
 */
#define TIMER_START                                       \
    {                                                     \
    ros::Time startTime = ros::Time::now();

/**
 * @brief Timer end macro
 * @details Using ROS time for timer, requires TIMER_START macro
 */
#define TIMER_END                                         \
    ros::Duration runTime = ros::Time::now() - startTime; \
    VETA_INFO("Run time >> %lf", runTime.toSec());        \
    }

#endif // VETA_NOETIC_UTIL_TIMER_H_
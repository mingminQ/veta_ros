/**
 * --------------------------------------------------
 *
 * @file    Color.h
 * @brief   Rviz Color Definition
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_NOETIC_RVIZ_COLOR_H_
#define VETA_NOETIC_RVIZ_COLOR_H_

#include "std_msgs/ColorRGBA.h"

namespace veta
{
    namespace rviz
    {
        // Rviz color yype
        enum RvizColor
        {
            BLACK   =  0,
            WHITE   =  1,
            RED     =  2,
            GREEN   =  3,
            BLUE    =  4,
            YELLOW  =  5,
            CYAN    =  6,
            MAGENTA =  7,
            PINK    =  8,
            ORANGE  =  9,
            PURPLE  = 10,
            COLOR_NUM

        }; // enum Color

        /**
         * @brief Rviz color function, color format is RGB and alpha
         * @param color Enum of the color
         * @param alpha Transparency of the color
         */
        std_msgs::ColorRGBA rvizColor(const int &color, const double &alpha = 1.0);

    } // namespace rviz

} // namespace veta

#endif // VETA_NOETIC_RVIZ_COLOR_H_
/**
 * --------------------------------------------------
 *
 * @file    Color.cpp
 * @brief   Rviz Color Definition Source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta_noetic/rviz/Color.h"
#include "veta/util/LogHandler.h"

namespace // anonymous
{
    // Convert RGBA data format ro ros message
    std_msgs::ColorRGBA toROSMessage(const double &r, const double &g, const double &b, const double &a)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;

        return color;
    }

} // namespace anonymous

/**
 * @brief Rviz color function
 * @details Color format is RGB and alpha
 * @param color Enum class of the color
 * @param alpha Transparency of the color
 */
std_msgs::ColorRGBA veta::rviz::rvizColor(const int &colorIdx, const double &alpha)
{
    switch(colorIdx)
    {
        case BLACK: // BLACK
            return toROSMessage(0.0, 0.0, 0.0, alpha);

        case WHITE: // WHITE
            return toROSMessage(1.0, 1.0, 1.0, alpha);

        case RED: // RED
            return toROSMessage(1.0, 0.0, 0.0, alpha);

        case GREEN: // GREEN
            return toROSMessage(0.0, 1.0, 0.0, alpha);

        case BLUE: // BLUE
            return toROSMessage(0.0, 0.0, 1.0, alpha);

        case YELLOW: // YELLOW
            return toROSMessage(1.0, 1.0, 0.0, alpha);

        case CYAN: // CYAN
            return toROSMessage(0.0, 1.0, 1.0, alpha);

        case MAGENTA: // MAGENTA
            return toROSMessage(1.0, 0.0, 1.0, alpha);

        case PINK: // PINK
            return toROSMessage(1.0, 0.75, 0.796, alpha);

        case ORANGE: // ORANGE
            return toROSMessage(1.0, 0.647, 0.0, alpha);

        case PURPLE: // PURPLE
            return toROSMessage(0.5, 0.0, 0.5, alpha);
    }

    VETA_ERROR("Invalid color, return defaut color [BLACK]");
    return toROSMessage(0.0, 0.0, 0.0, 1.0);
}
/**
 * --------------------------------------------------
 *
 * @file    Marker.h
 * @brief   Rviz Marker Utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_NOETIC_RVIZ_MARKER_H_
#define VETA_NOETIC_RVIZ_MARKER_H_

#include "veta_noetic/rviz/Color.h"
#include "visualization_msgs/Marker.h"

namespace veta
{
    namespace rviz
    {

        visualization_msgs::Marker rvizMarker();
        //visualization_msgs::MarkerArray rvizMarkerArray();

    } // namespace rviz

} // namespace veta

#endif // VETA_NOETIC_RVIZ_MARKER_H_
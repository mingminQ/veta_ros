/**
 * ------------------------------------------------------------
 *
 * @file    Point.cpp
 * @brief   Point utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * ------------------------------------------------------------
 */

#include "veta/util/LogHandler.h"
#include "veta/util/Exception.h"
#include "veta/math/Point.h"

#include <limits>

// "veta::Point" source

/** @brief Class constructor */
veta::Point::Point(const int &elementNum) : m_elementNum(elementNum)
{
}

/** @brief Class destructor */
veta::Point::~Point()
{
    delete m_elements;
}

/** @brief Point element access operator */
const double &veta::Point::operator[](const int &axis) const
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Point element segmentation fault, cannot access to point element");
    }
    return m_elements[axis];
}

/** @brief Point element access operator */
double &veta::Point::operator[](const int &axis)
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Point element segmentation fault, cannot access to point element");
    }
    return m_elements[axis];
}

/** @brief Point element setter function */
bool veta::Point::set(const int &axis, const double &value)
{
    if(axis < 0 || m_elementNum <= axis)
    {
        VETA_WARN("Point element segmentation fault, input is ignored");
        return false;
    }
    m_elements[axis] = value;
    return true;
}

/** @brief Exit condition of the recursion setter function */
bool veta::Point::set() const
{
    return true;
}

/** @brief Point element getter function */
double veta::Point::get(const int &axis) const
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Point element segmentation fault, cannot return element");
    }
    return m_elements[axis];
}

// "veta::Point2D" source

/** @brief Class constructor */
veta::Point2D::Point2D(const double &x, const double &y) : Point(Point2D::ELEMENT_NUM)
{
    m_elements = new double[Point2D::ELEMENT_NUM];
    m_elements[Point2D::X] = x;
    m_elements[Point2D::Y] = y;
}

// "veta::Point3D" source

/** @brief Class constructor */
veta::Point3D::Point3D(const double &x, const double &y, const double &z) : Point(Point3D::ELEMENT_NUM)
{
    m_elements = new double[Point3D::ELEMENT_NUM];
    m_elements[Point3D::X] = x;
    m_elements[Point3D::Y] = y;
    m_elements[Point3D::Z] = z;
}
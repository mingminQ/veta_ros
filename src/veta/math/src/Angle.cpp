/**
 * --------------------------------------------------
 *
 * @file    Angle.h
 * @brief   Angle utilty source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/util/LogHandler.h"
#include "veta/util/Exception.h"
#include "veta/math/Angle.h"

#include <cmath>

// "veta::Angle" source

/** @brief Class constructor */
veta::Angle::Angle(const int &elementNum) : m_elementNum(elementNum)
{
}

/** @brief Class destructor */
veta::Angle::~Angle()
{
    delete m_elements;
}

/** @brief Angle element access operator */
const double &veta::Angle::operator[](const int &axis) const
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Angle element segmentation fault, cannot access to point element");
    }
    return m_elements[axis];
}

/** @brief Angle element access operator */
double &veta::Angle::operator[](const int &axis)
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Angle element segmentation fault, cannot access to point element");
    }
    return m_elements[axis];
}

/** @brief Angle element setter function */
bool veta::Angle::set(const int &axis, const double &value)
{
    if(axis < 0 || m_elementNum <= axis)
    {
        VETA_WARN("Angle element segmentation fault, input is ignored");
        return false;
    }
    m_elements[axis] = value;
    return true;
}

/** @brief Exit condition of the recursion setter function */
bool veta::Angle::set() const
{
    return true;
}

/** @brief Angle element getter function */
double veta::Angle::get(const int &axis) const
{
    if(axis < 0 || m_elementNum <= axis)
    {
        Exception("Angle element segmentation fault, cannot return element");
    }
    return m_elements[axis];
}

// "veta::EulerRPY" source

/** @brief Class constructor */
veta::EulerRPY::EulerRPY(const double &roll, const double &pitch, const double &yaw)
  : Angle(EulerRPY::ELEMENT_NUM)
{
    m_elements = new double[EulerRPY::ELEMENT_NUM];
    m_elements[EulerRPY::ROLL]  = roll;
    m_elements[EulerRPY::PITCH] = pitch;
    m_elements[EulerRPY::YAW]   = yaw;
}

// "veta::Quaternion" source

/** @brief Class constructor */
veta::Quaternion::Quaternion(const double &qw, const double &qx, const double &qy, const double &qz)
  : Angle(Quaternion::ELEMENT_NUM)
{
    m_elements = new double[Quaternion::ELEMENT_NUM];
    m_elements[Quaternion::QW] = qw;
    m_elements[Quaternion::QX] = qx;
    m_elements[Quaternion::QY] = qy;
    m_elements[Quaternion::QZ] = qz;
}

// Angle function sources

/** @brief Convert degree to radians */
double veta::deg2rad(const double &deg)
{
    return deg * M_PI / 180.0;
}

/** @brief Convert radians to degree */
double veta::rad2deg(const double &rad)
{
    return rad * 180.0 / M_PI;
}

/** @brief Normalize radians by 2PI */
double veta::mod2PI(const double &rad)
{
    double result = std::fmod(rad, TWO_PI);

    if(result <= -M_PI)
    {
        result += TWO_PI;
    }
    else if(M_PI < result)
    {
        result -=TWO_PI;
    }
    return result;
}

/** @brief Convert euler angle to quaternion angle */
veta::Quaternion veta::rpy2quat(const double &roll, const double &pitch, const double &yaw)
{
    double cr = std::cos(roll * 0.5) , sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5)  , sy = std::sin(yaw * 0.5);
    
    return Quaternion(cr * cp * cy + sr * sp * sy
                     ,sr * cp * cy - cr * sp * sy
                     ,cr * sp * cy + sr * cp * sy
                     ,cr * cp * sy - sr * sp * cy );
}
inline
/** @brief Convert euler angle to quaternion angle */
veta::Quaternion veta::rpy2quat(const veta::EulerRPY &eulerRPY)
{
    return rpy2quat(eulerRPY.get(EulerRPY::ROLL)
                   ,eulerRPY.get(EulerRPY::PITCH)
                   ,eulerRPY.get(EulerRPY::YAW));
}

/** @brief Convert quaternion angle to euler angle */
veta::EulerRPY veta::quat2rpy(const double &w, const double &x, const double &y, const double &z)
{
    double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    double pitch = std::asin(2.0 * (w * y - z * x));
    double yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    return EulerRPY(roll, pitch, yaw);
}

/** @brief Convert quaternion angle to euler angle */
veta::EulerRPY veta::quat2rpy(const veta::Quaternion &quaternion)
{
    return quat2rpy(quaternion.get(Quaternion::QW)
                   ,quaternion.get(Quaternion::QX)
                   ,quaternion.get(Quaternion::QY)
                   ,quaternion.get(Quaternion::QZ));
}
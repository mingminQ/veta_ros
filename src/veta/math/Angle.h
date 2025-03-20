/**
 * --------------------------------------------------
 *
 * @file    Angle.h
 * @brief   Angle utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_MATH_ANGLE_H_
#define VETA_MATH_ANGLE_H_

#include "boost/concept_check.hpp"

namespace veta
{
    // Constant expression 2PI
    constexpr double TWO_PI = 6.28318530718;

    /** @brief Base class of the angle */
    class Angle
    {
    // "Angle" member variables
    protected:

        // Element number of the angle
        int m_elementNum;

        // Element values
        double *m_elements;

    // "Angle" member functions
    public:

        /** @brief Class constructor */
        Angle(const int &elementNum);

        /** @brief Class destructor */
        virtual ~Angle();

        /** @brief Angle const data type casting function */
        template <typename AngleType>
        const AngleType *angle_cast() const
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<AngleType *, Angle *>));
            return static_cast<const AngleType *>(this);
        }

        /** @brief Angle data type casting function */
        template <typename AngleType>
        AngleType *angle_cast()
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<AngleType *, Angle *>));
            return static_cast<AngleType *>(this);
        }

        /** @brief Angle element access operator */
        const double &operator[](const int &axis) const;

        /** @brief Angle element access operator */
        double &operator[](const int &axis);

        /** @brief Mutiple angle elements setter function */
        template <typename... Args>
        bool set(const int &axis, const double &value, Args... args)
        {
            bool isSet = set(axis, value);
            return isSet && set(args...);
        }

        /** @brief Angle element setter function */
        bool set(const int &axis, const double &value);

        /** @brief Exit condition of the recursion setter function */
        bool set() const;

        /** @brief Angle element getter function */
        double get(const int &axis) const;

    }; // class Angle

    /** @brief Euler angle roll, pitch, yaw class */
    class EulerRPY : public Angle
    {
    // "EulerRPY" member variables
    public:

        // "EulerRPY" elements
        enum Elements
        {
            ROLL  = 0,
            PITCH = 1,
            YAW   = 2,
            ELEMENT_NUM
        
        }; // enum Elements
    
    // "EulerRPY" member functions
    public:

        /** @brief Class constructor */
        EulerRPY(const double &roll, const double &pitch, const double &yaw);

        /** @brief Class destructor */
        ~EulerRPY() override = default;

    }; // class EulerRPY

    /** @brief Quaternion angle w, x, y, z class */
    class Quaternion : public Angle
    {
    // "Quaternion" member variables
    public:

        // "Quaternion" elements
        enum Elements
        {
            QW = 0,
            QX = 1,
            QY = 2,
            QZ = 3,
            ELEMENT_NUM

        }; // enum Elements

    // "Quaternion" member functions
    public:

        /** @brief Class constructor */
        Quaternion(const double &qw, const double &qx, const double &qy, const double &qz);

        /** @brief Class destructor */
        ~Quaternion() override = default;

    }; // class Quaternion

    // Angle functions

    /** @brief Convert degree to radians */
    double deg2rad(const double &deg);

    /** @brief Convert radians to degree */
    double rad2deg(const double &rad);

    /** @brief Normalize radians by 2PI */
    double mod2PI(const double &rad);

    /** @brief Convert euler angle to quaternion angle */
    Quaternion rpy2quat(const double &roll, const double &pitch, const double &yaw);

    /** @brief Convert euler angle to quaternion angle */
    Quaternion rpy2quat(const EulerRPY &eulerRPY);

    /** @brief Convert quaternion angle to euler angle */
    EulerRPY quat2rpy(const double &w, const double &x, const double &y, const double &z);

    /** @brief Convert euler angle to quaternion angle */
    EulerRPY quat2rpy(const Quaternion &quaternion);

} // namespace veta

#endif // VETA_UTIL_MATH_ANGLE_H_
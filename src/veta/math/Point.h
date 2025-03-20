/**
 * ------------------------------------------------------------
 *
 * @file    Point.h
 * @brief   Point utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * ------------------------------------------------------------
 */

#ifndef VETA_MATH_POINT_H_
#define VETA_MATH_POINT_H_

#include "boost/concept_check.hpp"

namespace veta
{
    /** @brief Base class of the class */
    class Point
    {
    // "Point" member variables
    protected:

        // Element number of the point
        int m_elementNum;

        // Element values
        double *m_elements;

    // "Point" member functions
    public:

        /** @brief Class constructor */
        Point(const int &elementNum);

        /** @brief Class destructor */
        virtual ~Point();

        /** @brief Point const data type casting function */
        template <typename PointType>
        const PointType *point_cast() const
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<PointType *, Point *>));
            return static_cast<const PointType *>(this);
        }

        /** @brief Point data type casting function */
        template <typename PointType>
        PointType *point_cast()
        {
            BOOST_CONCEPT_ASSERT((boost::Convertible<PointType *, Point *>));
            return static_cast<PointType *>(this);
        }

        /** @brief Point element access operator */
        const double &operator[](const int &axis) const;

        /** @brief Point element access operator */
        double &operator[](const int &axis);

        /** @brief Multiple point element setter function */
        template <typename... Args>
        bool set(const int &axis, const int &value, Args... args)
        {
            bool isSet = set(axis, value);
            return isSet && set(args...);
        }

        /** @brief Point element setter function */
        bool set(const int &axis, const double &value);
        
        /** @brief Exit condition of the recursion setter function */
        bool set() const;

        /** @brief Point element getter function */
        double get(const int &axis) const;
    
    }; // class Point

    /** @brief 2-Dimensional point class */
    class Point2D : public Point
    {
    // "Point2D" member functions
    public:

        // "Point2D" elements
        enum Elements
        {
            X = 1,
            Y = 2,
            ELEMENT_NUM
            
        }; // enum Elements

    // "Point2D" member functions
    public:

        /** @brief Class constructor */
        Point2D(const double &x, const double &y);

        /** @brief Class destructor */
        ~Point2D() override = default;

    }; // class Point2D

    /** @brief 3-Dimensional point class */
    class Point3D : public Point
    {
    // "Point3D" member functions
    public:

        // "Point3D" elements
        enum Elements
        {
            X = 1,
            Y = 2,
            Z = 3,
            ELEMENT_NUM

        }; // enum Elements

    // "Point3D" member functions
    public:

        /** @brief Class constructor */
        Point3D(const double &x, const double &y, const double &z);

        /** @brief Class destructor */
        ~Point3D() override = default;

    }; // class Point3D

} // namespace veta

#endif // VETA_MATH_POINT_H_
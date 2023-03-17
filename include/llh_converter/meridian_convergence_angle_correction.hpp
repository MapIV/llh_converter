// BSD 3-Clause License
//
// Copyright (c) 2023, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef M4_HEIGHT_CONVERTER_HPP
#define M4_HEIGHT_CONVERTER_HPP

#include <string>

#include <GeographicLib/Geoid.hpp>

#include "llh_converter/gsigeo.hpp"

#include <boost/math/constants/constants.hpp>

namespace llh_converter
{

enum class CoordinateSystem
{
    UTM = 0,
    MGRS = 1,
    PLANE = 2,
    LOCAL_CARTESIAN = 3,
};

struct GNSSStat
{
    GNSSStat()
        : coordinate_system(CoordinateSystem::MGRS), northup(true), zone(0), x(0), y(0), z(0), latitude(0),
          longitude(0), altitude(0)
    {
    }

    CoordinateSystem coordinate_system;
    bool northup;
    int zone;
    double x;
    double y;
    double z;
    double latitude;
    double longitude;
    double altitude;
};


struct Vector2d
{
    double x;
    double y;
};

double getDotNorm(Vector2d a, Vector2d b);

double getCrossNorm(Vector2d a, Vector2d b);

//void QuatMsg2RPY(const geometry_msgs::msg::Quaternion &quat_msg, double &roll, double &pitch, double &yaw);

//void RPY2QuatMsg(const double &roll, const double &pitch, const double &yaw, geometry_msgs::msg::Quaternion &quat_msg);

//geometry_msgs::msg::Quaternion RPY2QuatMsg(const double &roll, const double &pitch, const double &yaw);

template <class T> inline T square(T val)
{
    return val * val;
}

template <class T> inline T deg2rad(T deg)
{
    return deg * boost::math::constants::degree<T>();
}

template <class T> inline T deg2radSq(T deg)
{
    return deg * boost::math::constants::degree<T>() * boost::math::constants::degree<T>();
}

template <class T> inline T rad2deg(T rad)
{
    return rad * boost::math::constants::radian<T>();
}

// double getMeridianConvergence(const GNSSStat &lla, const GNSSStat &converted);

}  // namespace llh_converter

#endif

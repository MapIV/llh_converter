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

#include "llh_converter/meridian_convergence_angle_correction.hpp"

namespace llh_converter
{
double getDotNorm(Vector2d a, Vector2d b)
{
    return a.x * b.x + a.y * b.y;
}

double getCrossNorm(Vector2d a, Vector2d b)
{
    return a.x * b.y - a.y * b.x;
}

double getMeridianConvergence(const GNSSStat &lla, const GNSSStat &converted, llh_converter::LLHConverter &llhc,  const llh_converter::LLHParam &llhc_param)
{

    GNSSStat offset_lla = lla;
    GNSSStat offset_converted = converted;

    GNSSStat offset_lla_converted;

    offset_lla.latitude += 0.01;  // neary 1.11km
    offset_converted.y += 1000.0; // 1km

    llhc.convertDeg2XYZ(offset_lla.latitude, offset_lla.longitude, offset_lla.altitude, offset_lla_converted.x,
                         offset_lla_converted.y, offset_lla_converted.z, llhc_param);

    Vector2d offset_converted_vec;
    Vector2d offset_lla_converted_vec;

    offset_converted_vec.x = offset_converted.x - converted.x;
    offset_converted_vec.y = offset_converted.y - converted.y;
    offset_lla_converted_vec.x = offset_lla_converted.x - converted.x;
    offset_lla_converted_vec.y = offset_lla_converted.y - converted.y;

    double dot_norm = getDotNorm(offset_converted_vec, offset_lla_converted_vec);
    double cross_norm = getCrossNorm(offset_converted_vec, offset_lla_converted_vec);

    return atan2(cross_norm, dot_norm);
}

}  // namespace llh_converter

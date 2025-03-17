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
double getMeridianConvergence(const LLA& lla, const XYZ& xyz, LLHConverter& llhc, const LLHParam& llhc_param)
{
  LLA offset_lla = lla;
  XYZ offset_by_cartesian = xyz;

  XYZ offset_by_geodetic;

  offset_lla.latitude += 0.01;      // neary 1.11km. This value has no special meaning.
  offset_by_cartesian.y += 1000.0;  // 1km. This value has no special meaning.

  llhc.convertDeg2XYZ(offset_lla.latitude, offset_lla.longitude, offset_lla.altitude, offset_by_geodetic.x,
                      offset_by_geodetic.y, offset_by_geodetic.z, llhc_param);

  double cartesian_diff_x = offset_by_cartesian.x - xyz.x;
  double cartesian_diff_y = offset_by_cartesian.y - xyz.y;

  double geodetic_diff_x = offset_by_geodetic.x - xyz.x;
  double geodetic_diff_y = offset_by_geodetic.y - xyz.y;

  double dot_norm = cartesian_diff_x * geodetic_diff_x + cartesian_diff_y * geodetic_diff_y;
  double cross_norm = cartesian_diff_x * geodetic_diff_y - cartesian_diff_y * geodetic_diff_x;

  return std::atan2(cross_norm, dot_norm);
}

double getMeridianConvergence(const LLA& lla, LLHConverter& llhc, const LLHParam& llhc_param)
{
  XYZ xyz;
  llhc.convertDeg2XYZ(lla.latitude, lla.longitude, lla.altitude, xyz.x, xyz.y, xyz.z, llhc_param);

  return getMeridianConvergence(lla, xyz, llhc, llhc_param);
}

double getMeridicanConvergence(const XYZ& xyz, LLHConverter& llhc, const LLHParam& llhc_param)
{
  LLA lla;
  llhc.revertXYZ2Deg(xyz.x, xyz.y, lla.latitude, lla.longitude, llhc_param);
  lla.altitude = xyz.z;

  return getMeridianConvergence(lla, xyz, llhc, llhc_param);
}

}  // namespace llh_converter

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
#include "llh_converter/llh_converter.hpp"

#include <iostream>
#include <iomanip>

void test(const double& result, const double& answer)
{
  int i_result = std::round(result * 10000);
  int i_answer = std::round(answer * 10000);
  if (i_result == i_answer)
  {
    std::cout << "\033[32;1mTEST SUCCESS: " << result << " == " << answer << "\033[m" << std::endl;
  }
  else
  {
    std::cout << "\033[31;1mTEST FAILED : " << result << " != " << answer << "\033[m" << std::endl;
  }
}

int main(int argc, char** argv)
{
  // Meridian Convergence Angle Correction Test
  llh_converter::LLHConverter llh_converter;
  llh_converter::LLHParam param;
  param.use_mgrs = false;
  param.plane_num = 7;
  param.height_convert_type = llh_converter::ConvertType::NONE;
  param.geoid_type = llh_converter::GeoidType::EGM2008;

  // ref: Conversion to plane rectangular coordinates(in Japanse)
  // https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/bl2xyf.html
  // nagoya city ueda
  double test_lat = 35.141168610, test_lon = 136.989591759;
  double answered_angle = -0.101925000; // [deg]
  llh_converter::GNSSStat lla, converted;
  lla.latitude = test_lat;
  lla.longitude = test_lon;
  lla.altitude = 30.0;
  llh_converter.convertDeg2XYZ(lla.latitude, lla.longitude, lla.altitude, converted.x, converted.y, converted.z, param);
  double mca = llh_converter::getMeridianConvergence(lla, converted, llh_converter, param);
  std::cout << "Testing (" << std::setw(6) << test_lat << ", " << std::setw(6) << test_lat << ") ... " << std::endl;
  std::cout << "Meridian Convergence Angle (" << mca << ")" << std::endl;
  test(llh_converter::rad2deg(mca), answered_angle);

  return 0;
}
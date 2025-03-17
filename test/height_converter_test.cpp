// BSD 3-Clause License
//
// Copyright (c) 2022, Map IV, Inc.
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

#include "llh_converter/height_converter.hpp"
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

void test2(const double result0, const double result1, const double answer0, const double answer1)
{
  int i_result0 = std::round(result0 * 10000);
  int i_result1 = std::round(result1 * 10000);
  int i_answer0 = std::round(answer0 * 10000);
  int i_answer1 = std::round(answer1 * 10000);
  if (i_result0 == i_answer0 && i_result1 == i_answer1)
  {
    std::cout << "\033[32;1mTEST SUCCESS: " << result0 << ", " << result1 << " == " << answer0 << ", " << answer1
              << "\033[m" << std::endl;
  }
  else
  {
    std::cout << "\033[31;1mTEST FAILED : " << result0 << ", " << result1 << " != " << answer0 << ", " << answer1
              << "\033[m" << std::endl;
  }
}

int main(int argc, char** argv)
{
  llh_converter::HeightConverter hc;

  // GSIGEO Test
  std::cout << "GSIGEO2011 Test" << std::endl;
  hc.setGeoidType(llh_converter::GeoidType::GSIGEO2011);
  hc.loadGSIGEOGeoidFile();

  std::cout << "Testing (" << std::setw(6) << 35 << ", " << std::setw(6) << 135 << ") ... ";
  test(hc.getGeoidDeg(35, 135), 37.0557);

  std::cout << "Testing (" << std::setw(6) << 35.01 << ", " << std::setw(6) << 135.01 << ") ... ";
  test(hc.getGeoidDeg(35.01, 135.01), 37.0925);

  std::cout << "Testing (" << std::setw(6) << 35.5 << ", " << std::setw(6) << 135.5 << ") ... ";
  test(hc.getGeoidDeg(35.5, 135.5), 36.8691);

  std::cout << "Testing (" << std::setw(6) << 35 << ", " << std::setw(6) << 137 << ") ... ";
  test(hc.getGeoidDeg(35, 137), 38.4267);

  std::cout << "Testing (" << std::setw(6) << 39.2 << ", " << std::setw(6) << 140.9 << ") ... ";
  test(hc.getGeoidDeg(39.2, 140.9), 41.7035);

  std::cout << "Testing (" << std::setw(6) << 43.3 << ", " << std::setw(6) << 141.8 << ") ... ";
  test(hc.getGeoidDeg(43.3, 141.8), 31.3455);

  std::cout << "Testing (" << std::setw(6) << 43 << ", " << std::setw(6) << 141 << ") ... ";
  test(hc.getGeoidDeg(43, 141), 33.5306);

  std::cout << "Testing (" << std::setw(6) << 33.1 << ", " << std::setw(6) << 131.1 << ") ... ";
  test(hc.getGeoidDeg(33.1, 131.1), 33.0331);

  std::cout << "Testing (" << std::setw(6) << 26.6 << ", " << std::setw(6) << 128.1 << ") ... ";
  test(hc.getGeoidDeg(26.6, 128.1), 32.2696);

  // convertHeight Test
  test(hc.convertHeightDeg(35, 135, 0, llh_converter::ConvertType::ORTHO2ELLIPS), 37.0557);
  test(hc.convertHeightDeg(35, 135, 0, llh_converter::ConvertType::ELLIPS2ORTHO), -37.0557);

  // GSIGEO2024 Test
  std::cout << "GSIGEO2024 Test" << std::endl;
  hc.setGeoidType(llh_converter::GeoidType::GSIGEO2024);
  hc.loadGSIGEO2024GeoidFile();
  std::cout << "Testing (" << std::setw(9) << 36.104394 << ", " << std::setw(9) << 140.085365 << ") ... ";
  test(hc.getGeoidDeg(36.104394, 140.085365), 40.3059);
  std::cout << "Testing (" << std::setw(9) << 35.160410 << ", " << std::setw(9) << 139.615526 << ") ... ";
  test(hc.getGeoidDeg(35.160410, 139.615526), 36.7568);

  // LLHConverter test
  std::cout << "LLHConverter Test" << std::endl;
  llh_converter::LLHConverter llh_converter;
  llh_converter::LLHParam param;
  param.projection_method = llh_converter::ProjectionMethod::MGRS;
  param.height_convert_type = llh_converter::ConvertType::NONE;
  param.geoid_type = llh_converter::GeoidType::EGM2008;

  double test_lat = 35.5, test_lon = 135.5;

  double x, y, z;
  std::cout << "Testing MGRS  ... ";
  llh_converter.convertDeg2XYZ(test_lat, test_lon, 50, x, y, z, param);
  test2(x, y, 45346.7389, 28608.3575);

  // Test JPRCS
  param.projection_method = llh_converter::ProjectionMethod::JPRCS;
  param.grid_code = std::to_string(5);

  std::cout << "Testing JPRCS ... ";
  llh_converter.convertDeg2XYZ(test_lat, test_lon, 50, x, y, z, param);
  test2(x, y, 105842.7741, -54845.8269);

  // Test TM
  param.projection_method = llh_converter::ProjectionMethod::TM;
  param.tm_param.inv_flatten_ratio = 298.257222101;
  param.tm_param.semi_major_axis = 6378137.0;
  param.tm_param.scale_factor = 0.9996;
  param.tm_param.origin_lat_rad = 35.5 * M_PI / 180.;
  param.tm_param.origin_lon_rad = 135.5 * M_PI / 180.;

  std::cout << "Testing TM    ... ";
  llh_converter.convertDeg2XYZ(test_lat, test_lon, 50, x, y, z, param);
  test2(x, y, 0.0, 0.0);

  return 0;
}

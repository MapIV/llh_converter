// BSD 3-Clause License
//
// Copyright (c) 2024, Map IV, Inc.
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

/**
 * @file height_offset_test.cpp
 * @brief Test for height offset functionality in coordinate conversion
 * @test Verify that height_offset is correctly applied during coordinate transformation
 */

#include "llh_converter/llh_converter.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

// Utility function to compare floating-point values with tolerance
void testHeightOffset(const double& result, const double& expected, const std::string& test_name)
{
  const double tolerance = 1e-4;  // 0.1mm tolerance
  if (std::abs(result - expected) < tolerance)
  {
    std::cout << "\033[32;1m✓ PASS: " << test_name << " (result: " << std::fixed << std::setprecision(6)
              << result << " == expected: " << expected << ")\033[m" << std::endl;
  }
  else
  {
    std::cout << "\033[31;1m✗ FAIL: " << test_name << " (result: " << std::fixed << std::setprecision(6)
              << result << " != expected: " << expected << ", diff: " << (result - expected) << ")\033[m"
              << std::endl;
  }
}

int main()
{
  std::cout << "=== Height Offset Test ===" << std::endl << std::endl;

  // Test 1: Default behavior (no height offset)
  {
    std::cout << "Test 1: Default height_offset = 0.0 (No correction)" << std::endl;

    llh_converter::LLHConverter converter;
    llh_converter::LLHParam param_no_offset;
    param_no_offset.projection_method = llh_converter::ProjectionMethod::JPRCS;
    param_no_offset.grid_code = "7";
    param_no_offset.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
    param_no_offset.geoid_type = llh_converter::GeoidType::JPGEO2024;
    param_no_offset.orthometric_height_offset = 0.0;  // Default value

    double lat_deg = 35.6895;
    double lon_deg = 139.6917;
    double ellips_height = 10.0;

    double x, y, z;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x, y, z, param_no_offset);

    std::cout << "  Input: lat=" << lat_deg << "°, lon=" << lon_deg << "°, h=" << ellips_height << "m" << std::endl;
    std::cout << "  Output (no offset): X=" << x << ", Y=" << y << ", Z=" << z << std::endl;
    std::cout << std::endl;
  }

  // Test 2: Arakawa Peil (A.P.) correction
  {
    std::cout << "Test 2: Arakawa Peil (A.P.) height_offset = -1.134 m" << std::endl;

    llh_converter::LLHConverter converter;
    llh_converter::LLHParam param_no_offset;
    param_no_offset.projection_method = llh_converter::ProjectionMethod::JPRCS;
    param_no_offset.grid_code = "7";
    param_no_offset.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
    param_no_offset.geoid_type = llh_converter::GeoidType::JPGEO2024;
    param_no_offset.orthometric_height_offset = 0.0;

    llh_converter::LLHParam param_with_offset = param_no_offset;
    param_with_offset.orthometric_height_offset = -1.134;  // A.P. offset from T.P.

    double lat_deg = 35.6895;
    double lon_deg = 139.6917;
    double ellips_height = 10.0;

    double x1, y1, z1;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x1, y1, z1, param_no_offset);

    double x2, y2, z2;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x2, y2, z2, param_with_offset);

    std::cout << "  Input: lat=" << lat_deg << "°, lon=" << lon_deg << "°, h=" << ellips_height << "m" << std::endl;
    std::cout << "  T.P. Output:  Z=" << std::fixed << std::setprecision(4) << z1 << "m" << std::endl;
    std::cout << "  A.P. Output:  Z=" << std::fixed << std::setprecision(4) << z2 << "m" << std::endl;
    std::cout << "  Expected diff: -1.134m" << std::endl;
    std::cout << "  Actual diff:   " << (z2 - z1) << "m" << std::endl;

    testHeightOffset(z2 - z1, -1.134, "A.P. offset difference");
    std::cout << std::endl;
  }

  // Test 3: Positive offset (rare but valid case)
  {
    std::cout << "Test 3: Positive offset = 2.0 m (hypothetical datum higher than T.P.)" << std::endl;

    llh_converter::LLHConverter converter;
    llh_converter::LLHParam param_tp;
    param_tp.projection_method = llh_converter::ProjectionMethod::JPRCS;
    param_tp.grid_code = "7";
    param_tp.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
    param_tp.geoid_type = llh_converter::GeoidType::JPGEO2024;
    param_tp.orthometric_height_offset = 0.0;

    llh_converter::LLHParam param_positive = param_tp;
    param_positive.orthometric_height_offset = 2.0;

    double lat_deg = 35.6895;
    double lon_deg = 139.6917;
    double ellips_height = 10.0;

    double x1, y1, z1;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x1, y1, z1, param_tp);

    double x2, y2, z2;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x2, y2, z2, param_positive);

    std::cout << "  Input: lat=" << lat_deg << "°, lon=" << lon_deg << "°, h=" << ellips_height << "m" << std::endl;
    std::cout << "  T.P. Output:          Z=" << std::fixed << std::setprecision(4) << z1 << "m" << std::endl;
    std::cout << "  Positive +2.0 Output: Z=" << std::fixed << std::setprecision(4) << z2 << "m" << std::endl;

    testHeightOffset(z2 - z1, 2.0, "Positive offset difference");
    std::cout << std::endl;
  }

  // Test 4: Yokohama Port (Y.P.) offset
  {
    std::cout << "Test 4: Yokohama Port (Y.P.) height_offset = -1.09 m" << std::endl;

    llh_converter::LLHConverter converter;
    llh_converter::LLHParam param_tp;
    param_tp.projection_method = llh_converter::ProjectionMethod::JPRCS;
    param_tp.grid_code = "7";
    param_tp.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
    param_tp.geoid_type = llh_converter::GeoidType::JPGEO2024;
    param_tp.orthometric_height_offset = 0.0;

    llh_converter::LLHParam param_yp = param_tp;
    param_yp.orthometric_height_offset = -1.09;  // Y.P. offset from T.P.

    double lat_deg = 35.4437;
    double lon_deg = 139.6452;
    double ellips_height = 5.0;

    double x1, y1, z1;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x1, y1, z1, param_tp);

    double x2, y2, z2;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x2, y2, z2, param_yp);

    std::cout << "  Input: lat=" << lat_deg << "°, lon=" << lon_deg << "°, h=" << ellips_height << "m" << std::endl;
    std::cout << "  T.P. Output: Z=" << std::fixed << std::setprecision(4) << z1 << "m" << std::endl;
    std::cout << "  Y.P. Output: Z=" << std::fixed << std::setprecision(4) << z2 << "m" << std::endl;

    testHeightOffset(z2 - z1, -1.09, "Y.P. offset difference");
    std::cout << std::endl;
  }

  // Test 5: ELLIPS height type should NOT apply offset
  {
    std::cout << "Test 5: ELLIPS height type (offset should NOT be applied)" << std::endl;

    llh_converter::LLHConverter converter;
    llh_converter::LLHParam param_ellips;
    param_ellips.projection_method = llh_converter::ProjectionMethod::JPRCS;
    param_ellips.grid_code = "7";
    param_ellips.height_convert_type = llh_converter::ConvertType::NONE;  // ELLIPS
    param_ellips.geoid_type = llh_converter::GeoidType::JPGEO2024;
    param_ellips.orthometric_height_offset = -1.134;

    double lat_deg = 35.6895;
    double lon_deg = 139.6917;
    double ellips_height = 10.0;

    double x, y, z;
    converter.convertDeg2XYZ(lat_deg, lon_deg, ellips_height, x, y, z, param_ellips);

    std::cout << "  Input: lat=" << lat_deg << "°, lon=" << lon_deg << "°, h=" << ellips_height << "m" << std::endl;
    std::cout << "  Height type: ELLIPS (offset=-1.134 should be ignored)" << std::endl;
    std::cout << "  Output Z: " << std::fixed << std::setprecision(6) << z << "m" << std::endl;
    std::cout << "  Note: For ELLIPS type, output should match ellipsoidal height (no geoid or offset correction)"
              << std::endl;
    std::cout << std::endl;
  }

  std::cout << "=== All tests completed ===" << std::endl;
  return 0;
}

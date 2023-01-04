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

#include <iostream>

namespace llh_converter
{
static std::vector<std::string> geoid_strings = { "EGM2008", "GSIGEO2011", "NONE" };

std::string getStringFromGeoidType(const GeoidType& geoid)
{
  int enum_val = static_cast<int>(geoid);
  // 0: EGM2008, 1: GSIGEO2011
  return geoid_strings[enum_val];
}

GeoidType getGeoidTypeFromString(const std::string& geoid_string)
{
  for (int i = 0; i < geoid_strings.size(); i++)
  {
    if (geoid_strings[i] == geoid_string)
      return static_cast<GeoidType>(i);
  }

  std::cerr << "Error: No matching GeoidType: " << geoid_string << std::endl;
  exit(1);
}

double HeightConverter::convertHeightRad(const double& lat_rad, const double& lon_rad, const double& h,
                                         const ConvertType& type)
{
  double lat_deg = lat_rad * 180. / M_PI;
  double lon_deg = lon_rad * 180. / M_PI;
  return convertHeightDeg(lat_deg, lon_deg, h, type);
}

double HeightConverter::convertHeightDeg(const double& lat_deg, const double& lon_deg, const double& h,
                                         const ConvertType& type)
{
  double geoid_height = getGeoidDeg(lat_deg, lon_deg);
  return h + geoid_height * type;
}

void HeightConverter::setGeoidType(const GeoidType& geoid_type)
{
  geoid_type_ = geoid_type;
}

double HeightConverter::getGeoidRad(const double& lat_rad, const double& lon_rad)
{
  double lat_deg = lat_rad * 180. / M_PI;
  double lon_deg = lon_rad * 180. / M_PI;
  return getGeoidDeg(lat_deg, lon_deg);
}

double HeightConverter::getGeoidDeg(const double& lat_deg, const double& lon_deg)
{
  switch (geoid_type_)
  {
    case GeoidType::EGM2008:
      return getGeoidEGM2008(lat_deg, lon_deg);
    case GeoidType::GSIGEO2011:
      return getGeoidGSIGEO2011(lat_deg, lon_deg);
  }
}

void HeightConverter::loadGSIGEOGeoidFile(const std::string& geoid_file)
{
  gsigeo2011_.loadGeoidMap(geoid_file);
}

void HeightConverter::loadGSIGEOGeoidFile()
{
  gsigeo2011_.loadGeoidMap("/usr/share/GSIGEO/gsigeo2011_ver2_1.asc");
}

double HeightConverter::getGeoidEGM2008(const double& lat_deg, const double& lon_deg)
{
  try
  {
    return egm2008_(lat_deg, lon_deg);
  }
  catch (const GeographicLib::GeographicErr& e)
  {
    std::cerr << "GeographicLib Error: " << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }
}

double HeightConverter::getGeoidGSIGEO2011(const double& lat_deg, const double& lon_deg)
{
  return gsigeo2011_.getGeoid(lat_deg, lon_deg);
}
}  // namespace llh_converter

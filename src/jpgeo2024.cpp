// BSD 3-Clause License
//
// Copyright (c) 2025, Map IV, Inc.
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

#include "llh_converter/jpgeo2024.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <boost/algorithm/string.hpp>
namespace llh_converter
{
JPGEO2024::JPGEO2024()
{
}

JPGEO2024::~JPGEO2024()
{
}

void JPGEO2024::loadGeoidMap(const std::string& geoid_file)
{
  std::ifstream infile(geoid_file);
  if (!infile)
  {
    std::cerr << "Error opening file: " << geoid_file << std::endl;
    return;
  }

  std::string line;
  bool header_end = false;
  while (std::getline(infile, line))
  {
    if (line.find("end_of_head") != std::string::npos)
    {
      header_end = true;
      break;
    }
  }

  if (!header_end)
  {
    std::cerr << "Header not properly formatted in file." << std::endl;
    return;
  }

  geoid_map_.resize(row_size_, std::vector<double>(column_size_, -9999.0));

  for (int i = 0; i < row_size_; ++i)
  {
    for (int j = 0; j < column_size_; ++j)
    {
      if (!(infile >> geoid_map_[i][j]))
      {
        std::cerr << "Error reading geoid data at row " << i << ", col " << j << std::endl;
        return;
      }
    }
  }

  is_geoid_loaded_ = true;
}

void JPGEO2024::loadGeoidMap(const std::string& geoid_file, const std::string& hrefconv_file)
{
  loadGeoidMap(geoid_file);

  std::ifstream infile(hrefconv_file);
  if (!infile)
  {
    std::cerr << "Error opening file: " << hrefconv_file << std::endl;
    return;
  }

  std::string line;
  bool header_end = false;
  while (std::getline(infile, line))
  {
    if (line.find("end_of_head") != std::string::npos)
    {
      header_end = true;
      break;
    }
  }

  if (!header_end)
  {
    std::cerr << "Header not properly formatted in file." << std::endl;
    return;
  }

  for (int i = 0; i < row_size_; ++i)
  {
    for (int j = 0; j < column_size_; ++j)
    {
      double tmp;
      if (!(infile >> tmp))
      {
        std::cerr << "Error reading geoid data at row " << i << ", col " << j << std::endl;
        return;
      }

      if (tmp > -9998.0)
      {
        geoid_map_[i][j] += tmp;
      }
    }
  }
}

double JPGEO2024::getGeoid(const double& lat, const double& lon)
{
  if (!is_geoid_loaded_)
  {
    std::cerr << "Geoid map not loaded." << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (lat < 15.0 || lat > 50.0 || lon < 120.0 || lon > 160.0)
  {
    std::cerr << "Input coordinates out of range." << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Grid step (latitude 1 minute = 0.0166667 degrees, longitude 1.5 minutes = 0.025 degrees)
  const double lat_step = 1.0 / 60.0;
  const double lon_step = 1.5 / 60.0;

  // The index calculation from the reference point (latitude 50 degrees, longitude 120 degrees)
  int i1 = static_cast<int>((50.0 - lat) / lat_step);
  int j1 = static_cast<int>((lon - 120.0) / lon_step);

  int i2 = i1 + 1;
  int j2 = j1 + 1;

  if (i1 < 0 || i2 >= row_size_ || j1 < 0 || j2 >= column_size_)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Geoid height at the four corners
  double f00 = geoid_map_[i1][j1];
  double f10 = geoid_map_[i1][j2];
  double f01 = geoid_map_[i2][j1];
  double f11 = geoid_map_[i2][j2];

  if (f00 == -9999.0 || f10 == -9999.0 || f01 == -9999.0 || f11 == -9999.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Coordinates of the four corners
  double lat1 = 50.0 - i1 * lat_step;
  double lat2 = lat1 - lat_step;
  double lon1 = 120.0 + j1 * lon_step;
  double lon2 = lon1 + lon_step;

  // Calculate interpolation coefficients correctly
  double t = (lat1 - lat) / (lat1 - lat2);
  double u = (lon - lon1) / (lon2 - lon1);

  // Bilinear interpolation
  double interpolated_value = (1 - t) * (1 - u) * f00 + (1 - t) * u * f10 + t * (1 - u) * f01 + t * u * f11;

  return interpolated_value;
}

}  // namespace llh_converter

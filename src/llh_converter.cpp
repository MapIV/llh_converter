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

#include "llh_converter/llh_converter.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace llh_converter
{
template <typename L, typename R>
boost::bimaps::bimap<L, R>
makeBimap(std::initializer_list<typename boost::bimaps::bimap<L, R>::value_type> list)
{
    return boost::bimaps::bimap<L, R>(list.begin(), list.end());
}

// Constructor
LLHConverter::LLHConverter()
{
  height_converter_.loadGSIGEOGeoidFile();
  height_converter_.loadGSIGEO2024GeoidFile();
  initializeJPRCSOrigins();
  initializeMGRSAlphabet();
}

LLHConverter::LLHConverter(const std::string& geoid_file)
{
  height_converter_.loadGSIGEOGeoidFile(geoid_file);
  initializeJPRCSOrigins();
  initializeMGRSAlphabet();
}

// Public fumember functions
void LLHConverter::convertDeg2XYZ(const double& lat_deg, const double& lon_deg, const double& h, double& x, double& y,
                                  double& z, const LLHParam& param)
{
  double lat_rad = lat_deg * M_PI / 180.;
  double lon_rad = lon_deg * M_PI / 180.;

  convertRad2XYZ(lat_rad, lon_rad, h, x, y, z, param);
}

void LLHConverter::convertRad2XYZ(const double& lat_rad, const double& lon_rad, const double& h, double& x, double& y,
                                  double& z, const LLHParam& param)
{
  // Convert lat/lon to x/y
  switch (param.projection_method)
  {
    case ProjectionMethod::TM:
    {
      convRad2TM(lat_rad, lon_rad, param.tm_param, x, y);
      break;
    }
    case ProjectionMethod::JPRCS:
    {
      convRad2JPRCS(lat_rad, lon_rad, std::stoi(param.grid_code), x, y);
      break;
    }
    case ProjectionMethod::MGRS:
    {
      convRad2MGRS(lat_rad, lon_rad, x, y);
      break;
    }
    default:
    {
      std::cerr << "\033[31;1mError: Invalid projection method, " << static_cast<int>(param.projection_method) << "\033[0m" << std::endl;
      std::cerr << "\033[31;1mError: LLHConverter::convertRad2XYZ()\033[0m" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  // Convert h to z
  height_converter_.setGeoidType(param.geoid_type);
  z = height_converter_.convertHeightRad(lat_rad, lon_rad, h, param.height_convert_type);
}

void LLHConverter::revertXYZ2Deg(const double& x, const double& y, double& lat_deg, double& lon_deg,
                                 const LLHParam& param)
{
  double lat_rad = 0, lon_rad = 0;
  revertXYZ2Rad(x, y, lat_rad, lon_rad, param);
  lat_deg = lat_rad * 180. / M_PI;
  lon_deg = lon_rad * 180. / M_PI;
}

void LLHConverter::revertXYZ2Rad(const double& x, const double& y, double& lat_rad, double& lon_rad,
                                 const LLHParam& param)
{
  switch (param.projection_method)
  {
    case ProjectionMethod::TM:
    {
      revTM2Rad(x, y, param.tm_param, lat_rad, lon_rad);
      break;
    }
    case ProjectionMethod::JPRCS:
    {
      revJPRCS2Rad(x, y, std::stoi(param.grid_code), lat_rad, lon_rad);
      break;
    }
    case ProjectionMethod::MGRS:
    {
      revMGRS2Rad(x, y, param.grid_code, lat_rad, lon_rad);
      break;
    }
    default:
    {
      std::cerr << "\033[31;1mError: Invalid projection method, " << static_cast<int>(param.projection_method) << "\033[0m" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void LLHConverter::convertMGRS2JPRCS(const double& m_x, const double& m_y, double& j_x, double& j_y,
                                     const std::string& mgrs_code, const int jprcs_code)
{
  double lat_rad, lon_rad;
  revMGRS2Rad(m_x, m_y, mgrs_code, lat_rad, lon_rad);

  convRad2JPRCS(lat_rad, lon_rad, jprcs_code, j_x, j_y);
}

void LLHConverter::convertJPRCS2MGRS(const double& j_x, const double& j_y, double& m_x, double& m_y,
                                     const int jprcs_code)
{
  double lat_rad, lon_rad;
  revJPRCS2Rad(j_x, j_y, jprcs_code, lat_rad, lon_rad);

  convRad2MGRS(lat_rad, lon_rad, m_x, m_y);
}

void LLHConverter::getMapOriginDeg(double& lat_deg, double& lon_deg, const LLHParam& param)
{
  double lat_rad = 0, lon_rad = 0;
  getMapOriginRad(lat_rad, lon_rad, param);
  lat_deg = lat_rad * 180. / M_PI;
  lon_deg = lon_rad * 180. / M_PI;
}

void LLHConverter::getMapOriginRad(double& lat_rad, double& lon_rad, const LLHParam& param)
{
  switch (param.projection_method)
  {
    case ProjectionMethod::TM:
    {
      lat_rad = param.tm_param.origin_lat_rad;
      lon_rad = param.tm_param.origin_lon_rad;
      break;
    }
    case ProjectionMethod::JPRCS:
    {
      lat_rad = jprcs_origin_lat_rads_[std::stoi(param.grid_code)];
      lon_rad = jprcs_origin_lon_rads_[std::stoi(param.grid_code)];
      break;
    }
    case ProjectionMethod::MGRS:
    {
      revMGRS2Rad(0, 0, param.grid_code, lat_rad, lon_rad);
      break;
    }
    default:
    {
      std::cerr << "\033[31;1mError: Invalid projection method, " << static_cast<int>(param.projection_method) << "\033[0m" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

// Private Member functions
void LLHConverter::convRad2TM(const double& lat_rad, const double& lon_rad, const TMParam& param, double& x, double& y)
{
  double PS, PSo, PDL, Pt, PN, PW;

  double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
  double PA, PB, PC, PD, PE, PF_, PG, PH, PI;
  double Pe, Pet, Pnn;
  double F_W;

  F_W = 1.0 / param.inv_flatten_ratio;  // Geometrical flattening

  Pe = (double)sqrt(2.0 * F_W - pow(F_W, 2));
  Pet = (double)sqrt(pow(Pe, 2) / (1.0 - pow(Pe, 2)));

  PA = (double)1.0 + 3.0 / 4.0 * pow(Pe, 2) + 45.0 / 64.0 * pow(Pe, 4) + 175.0 / 256.0 * pow(Pe, 6) +
       11025.0 / 16384.0 * pow(Pe, 8) + 43659.0 / 65536.0 * pow(Pe, 10) + 693693.0 / 1048576.0 * pow(Pe, 12) +
       19324305.0 / 29360128.0 * pow(Pe, 14) + 4927697775.0 / 7516192768.0 * pow(Pe, 16);

  PB = (double)3.0 / 4.0 * pow(Pe, 2) + 15.0 / 16.0 * pow(Pe, 4) + 525.0 / 512.0 * pow(Pe, 6) +
       2205.0 / 2048.0 * pow(Pe, 8) + 72765.0 / 65536.0 * pow(Pe, 10) + 297297.0 / 262144.0 * pow(Pe, 12) +
       135270135.0 / 117440512.0 * pow(Pe, 14) + 547521975.0 / 469762048.0 * pow(Pe, 16);

  PC = (double)15.0 / 64.0 * pow(Pe, 4) + 105.0 / 256.0 * pow(Pe, 6) + 2205.0 / 4096.0 * pow(Pe, 8) +
       10395.0 / 16384.0 * pow(Pe, 10) + 1486485.0 / 2097152.0 * pow(Pe, 12) + 45090045.0 / 58720256.0 * pow(Pe, 14) +
       766530765.0 / 939524096.0 * pow(Pe, 16);

  PD = (double)35.0 / 512.0 * pow(Pe, 6) + 315.0 / 2048.0 * pow(Pe, 8) + 31185.0 / 131072.0 * pow(Pe, 10) +
       165165.0 / 524288.0 * pow(Pe, 12) + 45090045.0 / 117440512.0 * pow(Pe, 14) +
       209053845.0 / 469762048.0 * pow(Pe, 16);

  PE = (double)315.0 / 16384.0 * pow(Pe, 8) + 3465.0 / 65536.0 * pow(Pe, 10) + 99099.0 / 1048576.0 * pow(Pe, 12) +
       4099095.0 / 29360128.0 * pow(Pe, 14) + 348423075.0 / 1879048192.0 * pow(Pe, 16);

  PF_ = (double)693.0 / 131072.0 * pow(Pe, 10) + 9009.0 / 524288.0 * pow(Pe, 12) +
        4099095.0 / 117440512.0 * pow(Pe, 14) + 26801775.0 / 469762048.0 * pow(Pe, 16);

  PG = (double)3003.0 / 2097152.0 * pow(Pe, 12) + 315315.0 / 58720256.0 * pow(Pe, 14) +
       11486475.0 / 939524096.0 * pow(Pe, 16);

  PH = (double)45045.0 / 117440512.0 * pow(Pe, 14) + 765765.0 / 469762048.0 * pow(Pe, 16);

  PI = (double)765765.0 / 7516192768.0 * pow(Pe, 16);

  PB1 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PA;
  PB2 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PB / -2.0;
  PB3 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PC / 4.0;
  PB4 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PD / -6.0;
  PB5 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PE / 8.0;
  PB6 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PF_ / -10.0;
  PB7 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PG / 12.0;
  PB8 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PH / -14.0;
  PB9 = param.semi_major_axis * (1.0 - pow(Pe, 2)) * PI / 16.0;

  PS = (double)PB1 * lat_rad + PB2 * sin(2.0 * lat_rad) + PB3 * sin(4.0 * lat_rad) + PB4 * sin(6.0 * lat_rad) +
       PB5 * sin(8.0 * lat_rad) + PB6 * sin(10.0 * lat_rad) + PB7 * sin(12.0 * lat_rad) + PB8 * sin(14.0 * lat_rad) +
       PB9 * sin(16.0 * lat_rad);

  PSo = (double)PB1 * param.origin_lat_rad + PB2 * sin(2.0 * param.origin_lat_rad) + PB3 * sin(4.0 * param.origin_lat_rad) +
        PB4 * sin(6.0 * param.origin_lat_rad) + PB5 * sin(8.0 * param.origin_lat_rad) + PB6 * sin(10.0 * param.origin_lat_rad) +
        PB7 * sin(12.0 * param.origin_lat_rad) + PB8 * sin(14.0 * param.origin_lat_rad) + PB9 * sin(16.0 * param.origin_lat_rad);

  PDL = (double)lon_rad - param.origin_lon_rad;
  Pt = (double)tan(lat_rad);
  PW = (double)sqrt(1.0 - pow(Pe, 2) * pow(sin(lat_rad), 2));
  PN = param.semi_major_axis / PW;
  Pnn = (double)sqrt(pow(Pet, 2) * pow(cos(lat_rad), 2));

  y = (double)((PS - PSo) + (1.0 / 2.0) * PN * pow(cos(lat_rad), 2.0) * Pt * pow(PDL, 2.0) +
               (1.0 / 24.0) * PN * pow(cos(lat_rad), 4) * Pt *
                   (5.0 - pow(Pt, 2) + 9.0 * pow(Pnn, 2) + 4.0 * pow(Pnn, 4)) * pow(PDL, 4) -
               (1.0 / 720.0) * PN * pow(cos(lat_rad), 6) * Pt *
                   (-61.0 + 58.0 * pow(Pt, 2) - pow(Pt, 4) - 270.0 * pow(Pnn, 2) + 330.0 * pow(Pt, 2) * pow(Pnn, 2)) *
                   pow(PDL, 6) -
               (1.0 / 40320.0) * PN * pow(cos(lat_rad), 8) * Pt *
                   (-1385.0 + 3111 * pow(Pt, 2) - 543 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 8)) *
      param.scale_factor;

  x = (double)(PN * cos(lat_rad) * PDL -
               1.0 / 6.0 * PN * pow(cos(lat_rad), 3) * (-1 + pow(Pt, 2) - pow(Pnn, 2)) * pow(PDL, 3) -
               1.0 / 120.0 * PN * pow(cos(lat_rad), 5) *
                   (-5.0 + 18.0 * pow(Pt, 2) - pow(Pt, 4) - 14.0 * pow(Pnn, 2) + 58.0 * pow(Pt, 2) * pow(Pnn, 2)) *
                   pow(PDL, 5) -
               1.0 / 5040.0 * PN * pow(cos(lat_rad), 7) *
                   (-61.0 + 479.0 * pow(Pt, 2) - 179.0 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 7)) *
      param.scale_factor;
}

void LLHConverter::revTM2Rad(const double& x, const double& y, const TMParam& param, double& lat_rad, double& lon_rad)
{
  double n = 1. / (2 * param.inv_flatten_ratio - 1);
  double n2 = n * n;
  double n3 = n2 * n;
  double n4 = n3 * n;
  double n5 = n4 * n;
  double n6 = n5 * n;

  double A0 = 1 + n2 / 4. + n4 / 64.;
  double A1 = -3 / 2. * (n - n3 / 8. - n5 / 64.);
  double A2 = 15 / 16. * (n2 - n4 / 4.);
  double A3 = -35 / 48. * (n3 - 5 / 16. * n5);
  double A4 = 315 / 512. * n4;
  double A5 = -693 / 1280. * n5;

  double b1 = 1 / 2. * n - 2 / 3. * n2 + 37 / 96. * n3 - 1 / 360. * n4 - 81 / 512. * n5;
  double b2 = 1 / 48. * n2 + 1 / 15. * n3 - 437 / 1440. * n4 + 46 / 105. * n5;
  double b3 = 17 / 480. * n3 - 37 / 840. * n4 - 209 / 4480. * n5;
  double b4 = 4397 / 161280. * n4 - 11 / 504. * n5;
  double b5 = 4583 / 161280. * n5;

  double d1 = 2 * n - 2 / 3. * n2 - 2 * n3 + 116 / 45. * n4 + 26 / 45. * n5 - 2854 / 675. * n6;
  double d2 = 7 / 3. * n2 - 8 / 5. * n3 - 227 / 45. * n4 + 2704 / 315. * n5 + 2323 / 945. * n6;
  double d3 = 56 / 15. * n3 - 136 / 35. * n4 - 1262 / 105. * n5 + 73814 / 2835. * n6;
  double d4 = 4279 / 630. * n4 - 332 / 35. * n5 - 399572 / 14175. * n6;
  double d5 = 4174 / 315. * n5 - 144838 / 6237. * n6;
  double d6 = 601676 / 22275. * n6;

  double Sb = A0 * param.origin_lat_rad + A1 * std::sin(2 * param.origin_lat_rad) + A2 * std::sin(4 * param.origin_lat_rad) +
              A3 * std::sin(6 * param.origin_lat_rad) + A4 * std::sin(8 * param.origin_lat_rad) +
              A5 * std::sin(10 * param.origin_lat_rad);
  Sb = param.scale_factor * param.semi_major_axis / (1 + n) * Sb;

  double Ab = param.scale_factor * param.semi_major_axis / (1 + n) * A0;

  double eps = (y + Sb) / Ab;
  double eta = x / Ab;

  double eps1 = eps - b1 * std::sin(2 * eps) * std::cosh(2 * eta) - b2 * std::sin(4 * eps) * std::cosh(4 * eta) -
                b3 * std::sin(6 * eps) * std::cosh(6 * eta) - b4 * std::sin(8 * eps) * std::cosh(8 * eta) -
                b5 * std::sin(10 * eps) * std::cosh(10 * eta);
  double eta1 = eta - b1 * std::cos(2 * eps) * std::sinh(2 * eta) - b2 * std::cos(4 * eps) * std::sinh(4 * eta) -
                b3 * std::cos(6 * eps) * std::sinh(6 * eta) - b4 * std::cos(8 * eps) * std::sinh(8 * eta) -
                b5 * std::cos(10 * eps) * std::sinh(10 * eta);

  double X = std::asin(std::sin(eps1) / std::cosh(eta1));

  lat_rad = X + d1 * std::sin(2 * X) + d2 * std::sin(4 * X) + d3 * std::sin(6 * X) + d4 * std::sin(8 * X) +
            d5 * std::sin(10 * X) + d6 * std::sin(12 * X);
  lon_rad = param.origin_lon_rad + std::atan(std::sinh(eta1) / std::cos(eps1));
}

void LLHConverter::convRad2JPRCS(const double& lat_rad, const double& lon_rad, const int grid_code, double& x, double& y)
{
  TMParam tm_param = jprcs_tm_param_;
  tm_param.origin_lat_rad = jprcs_origin_lat_rads_[grid_code];
  tm_param.origin_lon_rad = jprcs_origin_lon_rads_[grid_code];

  convRad2TM(lat_rad, lon_rad, tm_param, x, y);
}

void LLHConverter::revJPRCS2Rad(const double& x, const double& y, const int grid_code, double& lat_rad, double& lon_rad)
{
  TMParam tm_param = jprcs_tm_param_;
  tm_param.origin_lat_rad = jprcs_origin_lat_rads_[grid_code];
  tm_param.origin_lon_rad = jprcs_origin_lon_rads_[grid_code];

  revTM2Rad(x, y, tm_param, lat_rad, lon_rad);
}

void LLHConverter::convRad2MGRS(const double& lat_rad, const double& lon_rad, double& x, double& y)
{
  int utm_zone;
  bool utm_northup;
  double utx, uty, utz;
  double lat_deg = lat_rad * 180. / M_PI;
  double lon_deg = lon_rad * 180. / M_PI;

  // LLH to UTM
  try
  {
    GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, utm_zone, utm_northup, utx, uty);
  }
  catch (const GeographicLib::GeographicErr& err)
  {
    std::cerr << "\033[31;1mGeographicLib Error: Failed to convert LLH to UTM: " << err.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  // UTM to MGRS
  int mgrs_zone;
  std::string mgrs_code;

  try
  {
    GeographicLib::MGRS::Forward(utm_zone, utm_northup, utx, uty, lat_deg, static_cast<int>(precision_), mgrs_code);
    mgrs_zone = std::stod(mgrs_code.substr(0, grid_code_size_));
    double map_rate = std::pow(10, static_cast<int>(MGRSPrecision::METER_1) - static_cast<int>(precision_));
    x = std::stod(mgrs_code.substr(grid_code_size_, static_cast<int>(precision_))) * map_rate;
    y = std::stod(mgrs_code.substr(grid_code_size_ + static_cast<int>(precision_), static_cast<int>(precision_))) *
        map_rate;

    if (use_origin_zone_)
    {
      if (!is_origin_set_)
      {
        mgrs_code_ = mgrs_code.substr(0, grid_code_size_);
        origin_x_zone_ = mgrs_code.substr(3, 1);
        origin_y_zone_ = mgrs_code.substr(4, 1);
        is_origin_set_ = true;
      }
      else
      {
        std::string x_zone = mgrs_code.substr(3, 1);
        std::string y_zone = mgrs_code.substr(4, 1);

        x += 100000 * checkCrossBoader(origin_x_zone_, x_zone, true);
        y += 100000 * checkCrossBoader(origin_y_zone_, y_zone, false);
      }
    }
  }
  catch (const GeographicLib::GeographicErr& err)
  {
    std::cerr << "\033[31;1mError: Failed to convert UTM to MGRS: " << err.what() << std::endl;
    exit(EXIT_FAILURE);
  }
}

void LLHConverter::revMGRS2Rad(const double& x, const double& y, std::string mgrs_code, double& lat_rad, double& lon_rad)
{
  auto [x_cross_num, x_in_grid] = getCrossNum(x);
  auto [y_cross_num, y_in_grid] = getCrossNum(y);

  if (x_cross_num != 0)
  {
    std::string x_zone = mgrs_code.substr(3, 1);
    x_zone = getOffsetZone(x_zone, x_cross_num);
    mgrs_code.replace(3, 1, x_zone);
  }
  if (y_cross_num != 0)
  {
    std::string y_zone = mgrs_code.substr(4, 1);
    y_zone = getOffsetZone(y_zone, y_cross_num);
    mgrs_code.replace(4, 1, y_zone);
  }

  std::ostringstream mgrs_x_code, mgrs_y_code;
  int digit_number = static_cast<int>(MGRSPrecision::MILLI_METER_10);
  mgrs_x_code << std::setw(digit_number) << std::setfill('0') << std::to_string(static_cast<int>(x_in_grid * 100));
  mgrs_y_code << std::setw(digit_number) << std::setfill('0') << std::to_string(static_cast<int>(y_in_grid * 100));

  std::string whole_mgrs_code = mgrs_code + mgrs_x_code.str() + mgrs_y_code.str();

  try
  {
    int zone, prec;
    bool northup;
    double utm_x, utm_y;
    double lat_deg, lon_deg;
    GeographicLib::MGRS::Reverse(whole_mgrs_code, zone, northup, utm_x, utm_y, prec);
    GeographicLib::UTMUPS::Reverse(zone, northup, utm_x, utm_y, lat_deg, lon_deg);
    lat_rad = lat_deg * M_PI / 180.;
    lon_rad = lon_deg * M_PI / 180.;
  }
  catch (const GeographicLib::GeographicErr& err)
  {
    std::cerr << "\033[31;1mGeographicLib Error: Failed to revert MGRS to lat/lon" << std::endl;
    exit(EXIT_FAILURE);
  }
}

std::pair<int, double> LLHConverter::getCrossNum(const double& x)
{
  int cross_num = 0;
  double ret_x = x;
  double grid_size = 100000; // 100km
  while (ret_x >= grid_size)
  {
    ret_x -= grid_size;
    cross_num++;
  }
  while (ret_x < 0)
  {
    ret_x += grid_size;
    cross_num--;
  }

  return std::make_pair(cross_num, ret_x);
}

std::string LLHConverter::getOffsetZone(const std::string& zone, const int& offset)
{
  if (offset == 0) return zone;

  int zone_num = mgrs_alphabet_.left.at(zone);
  zone_num += offset;
  if (zone_num > 23) zone_num -= 24;
  else if (zone_num < 0) zone_num += 24;

  return mgrs_alphabet_.right.at(zone_num);
}

int LLHConverter::checkCrossBoader(const std::string& code_origin, const std::string& code_current, bool is_x)
{
  int diff = mgrs_alphabet_.left.at(code_current) - mgrs_alphabet_.left.at(code_origin);

  if (is_x)
  {
    if (diff == -23 || diff == 1)
    {
      return 1;
    }
    else if (diff == 23 || diff == -1)
    {
      return -1;
    }
    else if (diff == 0)
    {
      return 0;
    }
    else
    {
      std::cerr << "\033[31;1mError: Straddling over 3 grids is not supported.\033[m" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  else  // is y
  {
    if (diff == -19 || diff == 1)
    {
      return 1;
    }
    else if (diff == 19 || diff == -1)
    {
      return -1;
    }
    else if (diff == 0)
    {
      return 0;
    }
    else
    {
      std::cerr << "\033[31;1mError: Straddling over 3 grids is not supported.\033[m" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

void LLHConverter::initializeJPRCSOrigins()
{
  // Initialize JPRCS origins
  // Empty element at index 0 to align with JPRCS grid code which starts from 1
  jprcs_origin_lat_rads_.resize(20);
  jprcs_origin_lon_rads_.resize(20);

  jprcs_origin_lat_rads_[1] = (33.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[1] = (129.0 + 30.0 / 60.0) / 180.0 * M_PI;
  
  jprcs_origin_lat_rads_[2] = (33.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[2] = (131.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[3] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[3] = (132.0 + 10.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[4] = (33.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[4] = (133.0 + 30.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[5] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[5] = (134.0 + 20.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[6] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[6] = (136.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[7] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[7] = (137.0 + 10.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[8] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[8] = (138.0 + 30.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[9] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[9] = (138.0 + 30.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[10] = (40.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[10] = (140.0 + 50.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[11] = (36.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[11] = (139.0 + 50.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[12] = (40.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[12] = (140.0 + 50.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[13] = (44.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[13] = (140.0 + 15.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[14] = (26.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[14] = (142.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[15] = (26.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[15] = (127.0 + 30.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[16] = (26.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[16] = (124.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[17] = (26.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[17] = (131.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[18] = (20.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[18] = (136.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_origin_lat_rads_[19] = (26.0 + 0.0 / 60.0) / 180.0 * M_PI;
  jprcs_origin_lon_rads_[19] = (154.0 + 0.0 / 60.0) / 180.0 * M_PI;

  jprcs_tm_param_.inv_flatten_ratio = 298.257222101;
  jprcs_tm_param_.semi_major_axis = 6378137;
  jprcs_tm_param_.scale_factor = 0.9999;
}

void LLHConverter::initializeMGRSAlphabet()
{
    mgrs_alphabet_ = makeBimap<std::string, int>({ { "A", 0 },  { "B", 1 },  { "C", 2 },  { "D", 3 },  { "E", 4 },
                                                   { "F", 5 },  { "G", 6 },  { "H", 7 },  { "J", 8 },  { "K", 9 },
                                                   { "L", 10 }, { "M", 11 }, { "N", 12 }, { "P", 13 }, { "Q", 14 },
                                                   { "R", 15 }, { "S", 16 }, { "T", 17 }, { "U", 18 }, { "V", 19 },
                                                   { "W", 20 }, { "X", 21 }, { "Y", 22 }, { "Z", 23 } });
}

double LLHConverter::getMeridianConvergenceRad(const double x, const double y, const LLHParam& param)
{
  double lat_rad = 0;
  double lon_rad = 0;

  revertXYZ2Rad(x, y, lat_rad, lon_rad, param);

  double cartesian_offset_x = x;
  double cartesian_offset_y = y + 100.0;

  double geodetic_offset_x = 0;
  double geodetic_offset_y = 0;
  double tmp_z = 0;

  convertRad2XYZ(lat_rad + 0.001, lon_rad, 0, geodetic_offset_x, geodetic_offset_y, tmp_z, param);

  double cartesian_diff_x = cartesian_offset_x - x;
  double cartesian_diff_y = cartesian_offset_y - y;

  double geodetic_diff_x = geodetic_offset_x - x;
  double geodetic_diff_y = geodetic_offset_y - y;

  double dot_norm = cartesian_diff_x * geodetic_diff_x + cartesian_diff_y * geodetic_diff_y;
  double cross_norm = cartesian_diff_x * geodetic_diff_y - cartesian_diff_y * geodetic_diff_x;

  return std::atan2(cross_norm, dot_norm);
}
}  // namespace llh_converter

# llh_converter

(Updated 2025/03/17)

This repository has two class implementation.

* HeightConverter
* LLHConverter

---

## HeightConverter

Convert height between ellipsoid and orthometric library

### Supported Geoid Models

* EGM2008-1
* GSIGEO2011 Ver2.1
* JPGEO2024 + Hrefconv2024

### Usage

```
llh_converter::HeightConverter hc;

hc.setGeoidType(height_converter::GeoidType::GSIGEO2011); // Select Geoid Model
// hc.setGeoidType(height_converter::GeoidType::EGM2008);

hc.setGSIGEOGeoidFile(path_to_gsigeo_asc_file);   // Load geoid data file when you select GSIGEO
// hc.setGSIGEOGeoidFile();  // If called with void, it try to read geoid data files under /usr/share/GSIGEO/

double geoid_heith = hc.getGeoid(lat, lon);   // Get geoid heigth with latitude/longitude in decimal degree

// Convert height between ellipsoid and orthometric
double converted_height = hc.convertHeight(lat, lon, h, height_converter::ConvertType::ORTHO2ELLIPS);
double converted_height = hc.convertHeight(lat, lon, h, height_converter::ConvertType::ELLIPS2ORTHO);
```

---

## LLHConverter

Convert latitude/longitude/altitude into XYZ coordinate system.

### Supported coordinate systems

* Millitary Grid Reference System (MGRS)
* Japan Plane Rectangular Coordinate System (JPRCS)
* Transverse Mercator with an arbitrary origin (TM)

### Usage

```
llh_converter::LLHConverter lc;
llh_converter::LLHParam param;              // parameter for conversion
param.projection_method = llh_converter::ProjectionMethod::TM;
                                            // set the projection method TM/JPRCS/MGRS
param.grid_code = "9";                        // set the grid code for JPRCS/MGRS
                                            // for MGRS, it's required only when reverting to lat/lon
param.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
                                            // You can also convert height
param.geoid_type = llh_converter::GeoidType::EGM2008;
                                            // Set geoid model
// The following tm_param is required only when the projection method is TM
param.tm_param.inv_flatten_ratio = 298.257222101;
                                            // Set the inverse flattening ratio
param.tm_param.semi_major_axis = 6378137.0; // Set the semi-major axis
param.tm_param.scale_factor = 0.9996;       // Set the scale factor
param.tm_param.origin_lat_rad = 35.0 * M_PI / 180.;
param.tm_param.origin_lon_rad = 139.0 * M_PI / 180.;
                                            // Set the origin

double lat_deg, lon_deg, alt;
double lat_rad = lat_deg * M_PI / 180.;
double lon_rad = lon_deg * M_PI / 180.;
double x, y, z;

lc.convertDeg2XYZ(lat_deg, lon_deg, alt, x, y, z, param);
lc.convertRad2XYZ(lat_rad, lon_rad, alt, x, y, z, param);

lc.revertXYZ2Deg(x, y, lat_deg, lon_deg, param);
lc.revertXYZ2Rad(x, y, lat_rad, lon_rad, param);
```

---

## meridian convergence angle correction

The meridian convergence angle is the angle of difference between true north and coordinate north.

The meridian convergence angle is calculated by the `getMeridianConvergence()` function.

<img src="docs/meridian_convergence_angle.png" width="750">

### Usage

```
  llh_converter::LLHConverter lc;
  llh_converter::LLHParam param;
  param.projection_method = llh_converter::ProjectionMethod::JPRCS;
  param.grid_code = "7";
  param.height_convert_type = llh_converter::ConvertType::NONE;
  param.geoid_type = llh_converter::GeoidType::EGM2008;

  llh_converter::LLA lla;
  llh_converter::XYZ xyz;
  lla.latitude = test_lat;
  lla.longitude = test_lon;
  lla.altitude = 30.0;
  llh_converter.convertDeg2XYZ(lla.latitude, lla.longitude, lla.altitude, xyz.x, xyz.y, xyz.z, param);
  double mca = llh_converter::getMeridianConvergence(lla, xyz, llh_converter, param); // meridian convergence angle
```

## Install

```
sudo apt update
sudo apt install libgeographic-dev geographiclib-tools geographiclib-doc

sudo geographiclib-get-geoids best

mkdir -p test_ws/src
cd test_ws/src/
git clone https://github.com/MapIV/llh_converter.git
sudo mkdir /usr/share/GSIGEO
sudo cp llh_converter/data/gsigeo2011_ver2_1.asc /usr/share/GSIGEO/
unzip llh_converter/data/JPGEO2024.zip
sudo mv JPGEO2024.isg Hrefconv2024.isg /usr/share/GSIGEO/
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Geoid model data

This package contains GSIGEO2011/JPGEO2024 geoid data file which is provided by Geospatial Information Authority of Japan.

[GSI's official website that relies on GSIGEO2011](https://fgd.gsi.go.jp/download/geoid.php)

[GSI's official website that relies on JPGEO2024](https://www.gsi.go.jp/buturisokuchi/grageo_reference.html)

## LICENSE

This package is provided under the [BSD 3-Clauses](LICENSE) License.

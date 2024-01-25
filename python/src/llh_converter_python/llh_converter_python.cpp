#include <llh_converter_python/llh_converter_python.h>

PYBIND11_MODULE(llh_converter, m) {
  m.doc() = "Python wrapper for llh_converter";

  // Wrapper for class GSIGEO2011
  py::class_<llh_converter::GSIGEO2011>(m, "GSIGEO2011")
      .def(py::init<>())
      .def("load_geoid_map", &llh_converter::GSIGEO2011::loadGeoidMap,
           py::arg("geoid_file"))
      .def("get_geoid", &llh_converter::GSIGEO2011::getGeoid);

  // Wrapper for enum class GeoidType
  py::enum_<llh_converter::GeoidType>(m, "GeoidType")
      .value("EGM2008", llh_converter::GeoidType::EGM2008)
      .value("GSIGEO2011", llh_converter::GeoidType::GSIGEO2011)
      .value("NONE", llh_converter::GeoidType::NONE)
      .export_values();

  // Utility functions for GeoidType
  m.def("getStringFromGeoidType", &llh_converter::getStringFromGeoidType);
  m.def("getGeoidTypeFromString", &llh_converter::getGeoidTypeFromString);

  // Wrapper for enum class ConvertType
  py::enum_<llh_converter::ConvertType>(m, "ConvertType")
      .value("ELLIPS2ORTHO", llh_converter::ConvertType::ELLIPS2ORTHO)
      .value("NONE", llh_converter::ConvertType::NONE)
      .value("ORTHO2ELLIPS", llh_converter::ConvertType::ORTHO2ELLIPS)
      .export_values();

  // Wrapper for class HeightConverter
  py::class_<llh_converter::HeightConverter>(m, "HeightConverter")
      .def(py::init<>())
      .def("convert_height_rad",
           &llh_converter::HeightConverter::convertHeightRad)
      .def("convert_height_deg",
           &llh_converter::HeightConverter::convertHeightDeg)
      .def("set_geoid_type", &llh_converter::HeightConverter::setGeoidType)
      .def("get_geoid_rad", &llh_converter::HeightConverter::getGeoidRad)
      .def("get_geoid_deg", &llh_converter::HeightConverter::getGeoidDeg)
      .def("load_gsigeo_geoid_file",
           py::overload_cast<const std::string &>(
               &llh_converter::HeightConverter::loadGSIGEOGeoidFile),
           py::arg("geoid_file"))
      .def("load_gsigeo_geoid_file",
           py::overload_cast<>(
               &llh_converter::HeightConverter::loadGSIGEOGeoidFile));

  // Wrapper for enum class MGRSPrecision
  py::enum_<llh_converter::MGRSPrecision>(m, "MGRSPrecision")
      .value("KILO_METER_10", llh_converter::MGRSPrecision::KILO_METER_10)
      .value("KILO_METER_1", llh_converter::MGRSPrecision::KILO_METER_1)
      .value("METER_100", llh_converter::MGRSPrecision::METER_100)
      .value("METER_10", llh_converter::MGRSPrecision::METER_10)
      .value("METER_1", llh_converter::MGRSPrecision::METER_1)
      .value("MILLI_METER_100", llh_converter::MGRSPrecision::MILLI_METER_100)
      .value("MILLI_METER_10", llh_converter::MGRSPrecision::MILLI_METER_10)
      .value("MILLI_METER_1", llh_converter::MGRSPrecision::MILLI_METER_1)
      .value("MICRO_METER_100", llh_converter::MGRSPrecision::MICRO_METER_100)
      .export_values();

  // Wrapper for LLHParam struct
  py::class_<llh_converter::LLHParam>(m, "LLHParam")
      .def(py::init<>())
      .def_readwrite("use_mgrs", &llh_converter::LLHParam::use_mgrs)
      .def_readwrite("plane_num", &llh_converter::LLHParam::plane_num)
      .def_readwrite("mgrs_code", &llh_converter::LLHParam::mgrs_code)
      .def_readwrite("height_convert_type",
                     &llh_converter::LLHParam::height_convert_type)
      .def_readwrite("geoid_type", &llh_converter::LLHParam::geoid_type);

  // Wrapper for class LLHConverter
  py::class_<llh_converter::LLHConverter>(m, "LLHConverter")
      .def(py::init<>())
      .def(py::init<const std::string &>())
      .def("convert_deg2xyz",
           [](llh_converter::LLHConverter &self, const double &lat_deg,
              const double &lon_deg, const double &h,
              const llh_converter::LLHParam &param) {
             double x, y, z;
             self.convertDeg2XYZ(lat_deg, lon_deg, h, x, y, z, param);
             return std::make_tuple(x, y, z);
           })
      .def("convert_rad2xyz",
           [](llh_converter::LLHConverter &self, const double &lat_rad,
              const double &lon_rad, const double &h,
              const llh_converter::LLHParam &param) {
             double x, y, z;
             self.convertRad2XYZ(lat_rad, lon_rad, h, x, y, z, param);
             return std::make_tuple(x, y, z);
           })
      .def("revert_xyz2deg",
           [](llh_converter::LLHConverter &self, const double &x,
              const double &y, const llh_converter::LLHParam &param) {
             double lat_deg, lon_deg;
             self.revertXYZ2Deg(x, y, lat_deg, lon_deg, param);
             return std::make_tuple(lat_deg, lon_deg);
           })
      .def("revert_xyz2rad",
           [](llh_converter::LLHConverter &self, const double &x,
              const double &y, const llh_converter::LLHParam &param) {
             double lat_rad, lon_rad;
             self.revertXYZ2Rad(x, y, lat_rad, lon_rad, param);
             return std::make_tuple(lat_rad, lon_rad);
           })
      .def("convert_mgrs2jprcs",
           [](llh_converter::LLHConverter &self, const double &m_x,
              const double &m_y, const llh_converter::LLHParam &param) {
             double j_x, j_y;
             self.convertMGRS2JPRCS(m_x, m_y, j_x, j_y, param);
             return std::make_tuple(j_x, j_y);
           })
      .def("convert_jprcs2mgrs",
           [](llh_converter::LLHConverter &self, const double &j_x,
              const double &j_y, const llh_converter::LLHParam &param) {
             double m_x, m_y;
             self.convertJPRCS2MGRS(j_x, j_y, m_x, m_y, param);
             return std::make_tuple(m_x, m_y);
           })
      .def("get_map_origin_deg",
           [](llh_converter::LLHConverter &self,
              const llh_converter::LLHParam &param) {
             double lat_deg, lon_deg;
             self.getMapOriginDeg(lat_deg, lon_deg, param);
             return std::make_tuple(lat_deg, lon_deg);
           })
      .def("get_map_origin_rad",
           [](llh_converter::LLHConverter &self,
              const llh_converter::LLHParam &param) {
             double lat_rad, lon_rad;
             self.getMapOriginRad(lat_rad, lon_rad, param);
             return std::make_tuple(lat_rad, lon_rad);
           })
      .def("get_mgrs_grid_code", &llh_converter::LLHConverter::getMGRSGridCode)
      .def("set_mgrs_grid_code", &llh_converter::LLHConverter::setMGRSGridCode);
}

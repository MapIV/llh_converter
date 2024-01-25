import llh_converter as llhc

# Set common parameters
mgrs_llh_converter = llhc.LLHConverter()
mgrs_param = llhc.LLHParam()
mgrs_param.use_mgrs = True
mgrs_param.mgrs_code = "53SPU"

llh_jp_converter = llhc.LLHConverter()
jp_param = llhc.LLHParam()
jp_param.use_mgrs = False
jp_param.plane_num = 7

# MGRS
lat, lon = mgrs_llh_converter.revert_xyz2deg(79175.234234, 91893.234723, mgrs_param)

# JP
x, y, z = llh_jp_converter.convert_deg2xyz(lat, lon, 0, jp_param)


print(x, y, z)

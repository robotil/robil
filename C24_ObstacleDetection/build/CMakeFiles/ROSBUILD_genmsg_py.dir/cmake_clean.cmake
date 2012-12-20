FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C24_ObstacleDetection/msg"
  "../src/C24_ObstacleDetection/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C24_ObstacleDetection/msg/__init__.py"
  "../src/C24_ObstacleDetection/msg/_C0C24_LAZ.py"
  "../src/C24_ObstacleDetection/msg/_C0C24_CAM.py"
  "../src/C24_ObstacleDetection/msg/_TBD.py"
  "../src/C24_ObstacleDetection/msg/_C24C0_ODIM.py"
  "../src/C24_ObstacleDetection/msg/_C0C24_SIZ.py"
  "../src/C24_ObstacleDetection/msg/_C24C0_OD.py"
  "../src/C24_ObstacleDetection/msg/_C0C24_AZI.py"
  "../src/C24_ObstacleDetection/msg/_C24C0_OPO.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

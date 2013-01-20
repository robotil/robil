FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/c21_Vision_and_Lidar/msg"
  "src/c21_Vision_and_Lidar/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/c21_Vision_and_Lidar/msg/__init__.py"
  "src/c21_Vision_and_Lidar/msg/_C0C21_CAM.py"
  "src/c21_Vision_and_Lidar/msg/_C0C21_AZI.py"
  "src/c21_Vision_and_Lidar/msg/_C0C21_RES.py"
  "src/c21_Vision_and_Lidar/msg/_C21C0_3DR.py"
  "src/c21_Vision_and_Lidar/msg/_C0C21_SIZ.py"
  "src/c21_Vision_and_Lidar/msg/_C21C0_3DF.py"
  "src/c21_Vision_and_Lidar/msg/_C0C21_LAZ.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

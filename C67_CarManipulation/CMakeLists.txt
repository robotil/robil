cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find gazebo include (FindPkgConfig)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# depends on DRCVehiclePlugin
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(Kinematics src/FK.cpp src/IK.cpp src/Path.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

rosbuild_add_executable(rPoint src/rPoint.cpp)
target_link_libraries(rPoint Kinematics)

rosbuild_add_executable(lPoint src/lPoint.cpp)
target_link_libraries(lPoint Kinematics)

rosbuild_add_executable(rPoint2 src/rPoint2.cpp)
target_link_libraries(rPoint2 Kinematics)

rosbuild_add_executable(lPoint2 src/lPoint2.cpp)
target_link_libraries(lPoint2 Kinematics)

rosbuild_add_executable(rPath src/rPath.cpp)
target_link_libraries(rPath Kinematics)

rosbuild_add_executable(lPath src/lPath.cpp)
target_link_libraries(lPath Kinematics)

rosbuild_add_executable(WheelMove src/WheelMove2.cpp)
target_link_libraries(WheelMove Kinematics)

rosbuild_add_executable(Ignition_server src/Ignition_server.cpp)
rosbuild_link_boost(Ignition_server thread)

rosbuild_add_executable(Ignition_client src/Ignition_client.cpp)
rosbuild_link_boost(Ignition_client thread)

rosbuild_add_executable(SteeringWheel_server src/SteeringWheel_server.cpp)
rosbuild_link_boost(SteeringWheel_server thread)

rosbuild_add_executable(SteeringWheel_client src/SteeringWheel_client.cpp)
rosbuild_link_boost(SteeringWheel_client thread)

rosbuild_add_executable(GasPedal_server src/GasPedal_server.cpp)
rosbuild_link_boost(GasPedal_server thread)

rosbuild_add_executable(GasPedal_client src/GasPedal_client.cpp)
rosbuild_link_boost(GasPedal_client thread)

rosbuild_add_executable(BrakePedal_server src/BrakePedal_server.cpp)
rosbuild_link_boost(BrakePedal_server thread)

rosbuild_add_executable(BrakePedal_client src/BrakePedal_client.cpp)
rosbuild_link_boost(BrakePedal_client thread)

rosbuild_add_executable(HandBrake_server src/HandBrake_server.cpp)
rosbuild_link_boost(HandBrake_server thread)

rosbuild_add_executable(HandBrake_client src/HandBrake_client.cpp)
rosbuild_link_boost(HandBrake_client thread)

rosbuild_add_executable(GearHandle_server src/GearHandle_server.cpp)
rosbuild_link_boost(GearHandle_server thread)

rosbuild_add_executable(GearHandle_client src/GearHandle_client.cpp)
rosbuild_link_boost(GearHandle_client thread)

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

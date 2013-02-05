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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

rosbuild_add_executable(Ignition_server src/Ignition_server.cpp)
rosbuild_link_boost(Ignition_server thread)

rosbuild_add_executable(Ignition_client src/Ignition_client.cpp)
rosbuild_link_boost(Ignition_client thread)

rosbuild_add_executable(SteeringWheel_server src/SteeringWheel_server.cpp)
rosbuild_link_boost(SteeringWheel_server thread)

rosbuild_add_executable(SteeringWheel_client src/SteeringWheel_client.cpp)
rosbuild_link_boost(SteeringWheel_client thread)

#rosbuild_add_executable(GasPedal_server src/GasPedal_server.cpp)
#rosbuild_link_boost(GasPedal_server thread)

#rosbuild_add_executable(BrakePedal_server src/BrakePedal_server.cpp)
#rosbuild_link_boost(BrakePedal_server thread)

#rosbuild_add_executable(HandBrake_server src/HandBrake_server.cpp)
#rosbuild_link_boost(HandBrake_server thread)

#rosbuild_add_executable(GearHandle_server src/GearHandle_server.cpp)
#rosbuild_link_boost(GearHandle_server thread)

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#---------- MAIN NODE -----------
rosbuild_add_executable(gpp src/gpp.cpp)
rosbuild_add_executable(gpp src/cogniteam_pathplanning.cpp)
rosbuild_add_executable(gpp src/PathPlanner.cpp)
rosbuild_add_executable(gpp src/PField.cpp)
#rosbuild_link_boost(gpp thread)

#---------- UTILS ---------------
rosbuild_add_executable(map_generator src/map_generator.cpp)
rosbuild_add_executable(map_generator src/cogniteam_pathplanning.cpp)
rosbuild_add_executable(map_generator src/PField.cpp)

#---------- TESTES --------------

rosbuild_add_executable(test_planner src/test_planning.cpp src/cogniteam_pathplanning.cpp src/PField.cpp)
rosbuild_add_executable(test_addPointsToPath src/test_addPointsToPath.cpp src/cogniteam_pathplanning.cpp src/PField.cpp)
rosbuild_add_executable(test_planner_transit src/test_planning_with_transit.cpp src/cogniteam_pathplanning.cpp src/PField.cpp)

#---------- SIMULATORS ----------
rosbuild_add_executable(test_c11_simulator src/test_c11_simulator.cpp)


#rosbuild_add_executable(lpp src/lpp.cpp)
#rosbuild_link_boost(lpp thread)


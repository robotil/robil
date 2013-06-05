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
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


find_package( PCL 1.5 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rosbuild_add_boost_directories()
find_package(OpenCV REQUIRED)
rosbuild_add_executable(C21 src/C21_Node.cpp)
rosbuild_link_boost(C21 thread signals)
target_link_libraries(C21 ${OpenCV_LIBS} ${PCL_LIBRARIES})
rosbuild_add_executable(test1 src/C21SceneTest.cpp)
rosbuild_link_boost(test1 thread signals)
target_link_libraries(test1 ${OpenCV_LIBS} ${PCL_LIBRARIES})
rosbuild_add_executable(test2 src/C21ImageTest.cpp)
rosbuild_link_boost(test2 thread signals)
target_link_libraries(test2 ${OpenCV_LIBS} ${PCL_LIBRARIES})
rosbuild_add_executable(test3 src/C21PanoramaTest.cpp)
rosbuild_link_boost(test3 thread signals)
target_link_libraries(test3 ${OpenCV_LIBS} ${PCL_LIBRARIES})
rosbuild_add_executable(test4 src/C21ImageServiceTest.cpp)
rosbuild_link_boost(test4 thread signals)
target_link_libraries(test4 ${OpenCV_LIBS} ${PCL_LIBRARIES})
rosbuild_add_executable(test5 src/C21_ObjectLocationTest.cpp)
rosbuild_link_boost(test5 thread signals)
target_link_libraries(test5 ${OpenCV_LIBS} ${PCL_LIBRARIES})

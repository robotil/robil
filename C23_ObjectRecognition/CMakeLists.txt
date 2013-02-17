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


#find_package( PCL 1.5 REQUIRED )
add_library(libopentld.a STATIC IMPORTED)
set_property(TARGET libopentld.a PROPERTY IMPORTED_LOCATION libopentld.a)
include_directories(${PCL_INCLUDE_DIRS} src/OpenTLD/libopentld/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
rosbuild_add_boost_directories()
find_package(OpenCV REQUIRED)
rosbuild_add_executable(c23 src/C23_Node.cpp)
rosbuild_link_boost(c23 thread signals)
include_directories(main
    src/libopentld/imacq
    src/libopentld/mftracker
    src/libopentld/tld
    src/3rdparty/cvblobs
    ${OpenCV_INCLUDE_DIRS})


#-------------------------------------------------------------------------------
#add subdirectories
add_subdirectory(src/3rdparty/cvblobs)

if(NOT USE_SYSTEM_LIBS)
    add_subdirectory(src/3rdparty/libconfig)
endif(NOT USE_SYSTEM_LIBS)

add_subdirectory(src/libopentld)

target_link_libraries(c23 ${PCL_LIBRARIES} libopentld cvblobs ${OpenCV_LIBS} )
rosbuild_add_executable(c23 src/C23_Node.cpp src/C23_Node_TLD_Handler.cpp)
#rosbuild_add_executable(test1 src/C21testNode.cpp)
#rosbuild_link_boost(test1 thread signals)
#target_link_libraries(test1 ${OpenCV_LIBS} ${PCL_LIBRARIES})

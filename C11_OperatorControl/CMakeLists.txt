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

find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
set( QT_USE_QTNETWORK TRUE )
include(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
set(qt_hdrs src/graphicsview.h src/imagedraw.h src/C11_Node.h src/mapmain.h src/mapwidget.h src/pixitem.h src/routeitem.h src/lineItemlist.h src/lineitem.h src/pointitem.h src/structs.h src/arcitem.h src/polygonitem.h src/traingleitem.h src/tcpConnection.h)
set(qt_srcs src/graphicsview.cpp src/imagedraw.cpp src/main.cpp src/C11_Node.cpp src/mapmain.cpp src/mapwidget.cpp src/pixitem.cpp src/routeitem.cpp src/lineitemlist.cpp src/lineitem.cpp src/pointitem.cpp src/arcitem.cpp src/polygonitem.cpp src/traingleitem.cpp src/tcpConnection.cpp src/structs.cpp)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

QT4_WRAP_UI(uis_h src/imagedraw.ui src/mapmain.ui)

# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(src)
include_directories(srv_gen/cpp/include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_boost_directories()
#rosbuild_add_executable(C11 src/C11_Node.cpp)
rosbuild_add_executable(C11 src/main.cpp
    ${uis_h} ${qt_hdrs} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(C11 ${QT_LIBRARIES})
rosbuild_link_boost(C11 thread signals)

cmake_minimum_required(VERSION 3.10)
project(commen_tool)

set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Ofast -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
#SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(${PCL_DEFINITIONS})
#add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(Yaml REQUIRED)
#find_package(Ceres REQUIRED)
find_package(PCL  REQUIRED)
find_package(Ceres REQUIRED)
#find_package(Glog REQUIRED)
#find_package(gflags REQUIRED)
add_definitions(${PCL_DEFINITIONS})

include_directories(app src)
include_directories(
       # ${gflags_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
       # ${GLOG_INCLUDE_DIRS}
)

add_library(auto_calibration
       src/clustering.cpp
       src/plane_segmentation.cpp
       src/rdp.cpp)


add_executable(point_cloud_cut app/point_cloud_cut.cpp)
target_link_libraries(point_cloud_cut
        gflags
        ${Eigen3_LIBRARIES}
        #        ${OpenCV_LIBRARIES}
        #${YAMLCPP_LIBRARIES}
        #        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        )


add_executable(plane_segmentation_main app/plane_segmentation_main.cpp)
target_link_libraries(plane_segmentation_main
        auto_calibration
        gflags
        glog
        ${Eigen3_LIBRARIES}
        #        ${OpenCV_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(cloud_cut app/cloud_cut.cpp)
target_link_libraries(cloud_cut
        auto_calibration
        gflags
        glog
        ${Eigen3_LIBRARIES}
        #        ${OpenCV_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        )

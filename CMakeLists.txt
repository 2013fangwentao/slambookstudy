cmake_minimum_required(VERSION 3.5)

project(vSLAMLearning_v1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_COMPILER "g++")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#Eigen
include_directories("/usr/include/eigen3")
#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

set(
    THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
################## source and test#####################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )



cmake_minimum_required(VERSION 3.16)
project(dvxplore_stream C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_path(LIBCAER_INCLUDE_DIR NAMES libcaer/libcaer.h
  PATHS ${CMAKE_SOURCE_DIR}/external/libcaer-prebuilt/include)
find_library(LIBCAER_LIB NAMES caer
  PATHS ${CMAKE_SOURCE_DIR}/external/libcaer-prebuilt/lib)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_path(EIGEN_INCLUDE_DIR NAMES Eigen/Dense PATHS /usr/include/eigen3 /home/thomas/eigen)

add_executable(dvxplore_simple  src/dvx_drivers.cpp third_party/fast_detector.cpp)
set_target_properties(dvxplore_simple PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(dvxplore_simple PRIVATE
  src/ ${LIBCAER_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

target_link_libraries(dvxplore_simple PRIVATE
 ${LIBCAER_LIB} pthread rt ${OpenCV_LIBS} ${EIGEN_LIBS})
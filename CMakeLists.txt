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
find_path(EIGEN_INCLUDE_DIR NAMES Eigen/Dense PATHS /usr/include/eigen3)

# evio_core algorithm itself
add_library(evio_core STATIC
  src/RawAdapter.cpp
  src/Feature.cpp
  src/FeatureDetector.cpp
  src/OpticalFlow.cpp
  src/TemplateAlignment.cpp
  src/OutlierRejection.cpp
  src/TemporalWindow.cpp
  src/FeatureTracker.cpp
  src/StateEstimator.cpp
  src/EVIO.cpp
)

target_include_directories(evio_core PUBLIC
  include/
  third_party/
  ${EIGEN_INCLUDE_DIR}
  ${LIBCAER_INCLUDE_DIR}
)

# dvxplore_simple
add_executable(dvxplore_simple
  src/DVX_Drivers.cpp
  third_party/fast_detector.cpp
  src/main.cpp
)

target_include_directories(dvxplore_simple PRIVATE
  ${LIBCAER_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(dvxplore_simple PRIVATE
  evio_core
  ${LIBCAER_LIB}
  pthread
  rt
  ${OpenCV_LIBS}
)

# unit tests
add_executable(test_feature
  unit_tests/test_feature.cpp
)
target_link_libraries(test_feature PRIVATE evio_core)
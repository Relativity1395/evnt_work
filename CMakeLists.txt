cmake_minimum_required(VERSION 3.16)
project(dvxplore_stream C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

# Export compile_commands.json so clangd uses proper Linux headers
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── libcaer ───────────────────────────────────────────────────────────────────
# Prefer the local prebuilt tree; fall back to the system-installed library.
find_path(LIBCAER_INCLUDE_DIR
    NAMES libcaer/libcaer.h
    PATHS ${CMAKE_SOURCE_DIR}/external/libcaer-prebuilt/include
)

find_library(LIBCAER_LIB
    NAMES caer
    PATHS ${CMAKE_SOURCE_DIR}/external/libcaer-prebuilt/lib
)

if(NOT LIBCAER_INCLUDE_DIR OR NOT LIBCAER_LIB)
    message(STATUS "Local libcaer-prebuilt not found — searching system paths")
    find_path(LIBCAER_INCLUDE_DIR NAMES libcaer/libcaer.h REQUIRED)
    find_library(LIBCAER_LIB NAMES caer REQUIRED)
endif()

message(STATUS "libcaer include : ${LIBCAER_INCLUDE_DIR}")
message(STATUS "libcaer library : ${LIBCAER_LIB}")

# ── OpenCV (for visualizer) ───────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
message(STATUS "OpenCV version  : ${OpenCV_VERSION}")

# ── Static library: core event/flow/DMA helpers ───────────────────────────────
add_library(dvxplore_core STATIC
    src/event_pack.c
    src/flow_unpack.c
    # src/dma.c
)
target_include_directories(dvxplore_core PUBLIC src/)

# ── Static library: OpenCV visualizer (C++) ───────────────────────────────────
# add_library(dvxplore_viz STATIC
#     src/visualizer.cpp
# )
# target_include_directories(dvxplore_viz PUBLIC src/)
# target_link_libraries(dvxplore_viz PUBLIC ${OpenCV_LIBS})

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(dvxplore_simple src/dvxplore_simple.cpp)

# The main source is C but links a C++ library; use the C++ linker to pull in
# libstdc++ / libm automatically.
set_target_properties(dvxplore_simple PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(dvxplore_simple PRIVATE
    src/
    ${LIBCAER_INCLUDE_DIR}
    ${OpenCV_LIBS}
)

target_link_libraries(dvxplore_simple PRIVATE
    dvxplore_core
    # dvxplore_viz
    ${LIBCAER_LIB}
    pthread
    rt
    ${OpenCV_LIBS}
)

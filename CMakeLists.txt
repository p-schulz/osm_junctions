cmake_minimum_required(VERSION 3.16)
project(osm2xodr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION "Build with libOpenDRIVE-based --validate support" OFF)
option(OSM2XODR_BUILD_TESTS "Build the osm2xodr test suite" ON)

# --- libosmium / protozero (header-only) -----------------------------------
add_library(osmium_headers INTERFACE)
target_include_directories(osmium_headers INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/external/libosmium/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/protozero/include
)

find_package(EXPAT REQUIRED)
find_package(ZLIB REQUIRED)
find_library(BZ2_LIBRARY NAMES bz2 bzip2)
if(NOT BZ2_LIBRARY)
    message(FATAL_ERROR "libbz2 not found (required by libosmium's PBF reader)")
endif()
find_package(Threads REQUIRED)

target_link_libraries(osmium_headers INTERFACE EXPAT::EXPAT ZLIB::ZLIB ${BZ2_LIBRARY} Threads::Threads)

# --- osm2xodr core library (shared model/geo/tags/util headers) ------------
add_library(osm2xodr_core INTERFACE)
target_include_directories(osm2xodr_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(osm2xodr_core INTERFACE osmium_headers)

if(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION)
    set(OPENDRIVE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(external/libOpenDRIVE)
endif()

# --- legacy tag-inference road generator (src/model_builder.cpp, src/infer.cpp) ---
# STALE: this pipeline is superseded by the OSM-control-line/control-point procedural
# generator in src/procedural/. It is kept buildable only so the existing `osm2xodr`
# CLI keeps working during the deprecation period; see src/model_builder.cpp and
# src/infer.cpp for the stale banner. Do not extend this pipeline further.
add_library(osm2xodr_legacy STATIC
    src/config.cpp
    src/infer.cpp
    src/model_builder.cpp
    src/osm_parse.cpp
    src/report.cpp
    src/xodr_writer.cpp
)
target_link_libraries(osm2xodr_legacy PUBLIC osm2xodr_core)

add_executable(osm2xodr src/main.cpp src/cli.cpp)
target_link_libraries(osm2xodr PRIVATE osm2xodr_legacy)
if(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION)
    target_link_libraries(osm2xodr PRIVATE OpenDrive::OpenDrive)
    target_compile_definitions(osm2xodr PRIVATE OSM2XODR_WITH_LIBOPENDRIVE)
endif()

# --- procedural OSM-control-line/control-point road-network generator ------
add_library(osm2xodr_procedural STATIC
    src/osm_parse.cpp
    src/xodr_writer.cpp
    src/procedural/road_type_shim.cpp
    src/procedural/geo_extra.cpp
    src/procedural/control_lines.cpp
    src/procedural/control_points.cpp
    src/procedural/grouping.cpp
    src/procedural/graph.cpp
    src/procedural/lanes.cpp
    src/procedural/intersections.cpp
    src/procedural/cleanup.cpp
    src/procedural/model_assembly.cpp
    src/procedural/geojson_debug.cpp
    src/procedural/pipeline.cpp
    src/procedural/cli.cpp
)
target_link_libraries(osm2xodr_procedural PUBLIC osm2xodr_core)
target_include_directories(osm2xodr_procedural PUBLIC include)
if(OSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION)
    target_link_libraries(osm2xodr_procedural PUBLIC OpenDrive::OpenDrive)
    target_compile_definitions(osm2xodr_procedural PUBLIC OSM2XODR_WITH_LIBOPENDRIVE)
endif()

add_executable(osm2xodr_procedural_cli src/procedural/main.cpp)
target_link_libraries(osm2xodr_procedural_cli PRIVATE osm2xodr_procedural)
set_target_properties(osm2xodr_procedural_cli PROPERTIES OUTPUT_NAME osm2xodr-procedural)

if(MSVC)
    target_compile_options(osm2xodr_legacy PRIVATE /W3)
    target_compile_options(osm2xodr_procedural PRIVATE /W3)
else()
    target_compile_options(osm2xodr_legacy PRIVATE -Wall -Wextra)
    target_compile_options(osm2xodr_procedural PRIVATE -Wall -Wextra)
endif()

if(OSM2XODR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

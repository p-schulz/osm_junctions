cmake_minimum_required(VERSION 3.16)
project(XOSM LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(XOSM_ENABLE_LIBOPENDRIVE_VALIDATION "Build with libOpenDRIVE-based --validate support" OFF)
option(XOSM_BUILD_TESTS "Build the XOSM test suite" ON)
option(XOSM_BUILD_GUI "Build the imgui-based xosm-gui viewer (fetches GLFW)" ON)

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

# --- xosm core library (shared model/geo/tags/util headers) ----------------
add_library(xosm_core INTERFACE)
target_include_directories(xosm_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(xosm_core INTERFACE osmium_headers)

if(XOSM_ENABLE_LIBOPENDRIVE_VALIDATION)
    set(OPENDRIVE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(external/libOpenDRIVE)
endif()

# --- procedural OSM-control-line/control-point road-network generator ------
add_library(xosm_procedural STATIC
    src/osm_parse.cpp
    src/xodr_writer.cpp
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
target_link_libraries(xosm_procedural PUBLIC xosm_core)
target_include_directories(xosm_procedural PUBLIC include)
if(XOSM_ENABLE_LIBOPENDRIVE_VALIDATION)
    target_link_libraries(xosm_procedural PUBLIC OpenDrive::OpenDrive)
    target_compile_definitions(xosm_procedural PUBLIC XOSM_WITH_LIBOPENDRIVE)
endif()

add_executable(xosm_cli src/procedural/main.cpp)
target_link_libraries(xosm_cli PRIVATE xosm_procedural)
set_target_properties(xosm_cli PROPERTIES OUTPUT_NAME xosm)

if(MSVC)
    target_compile_options(xosm_procedural PRIVATE /W3)
else()
    target_compile_options(xosm_procedural PRIVATE -Wall -Wextra)
endif()

# --- GUI viewer: imgui (external/imgui) + GLFW/OpenGL3 backend + tinyfiledialogs ---
if(XOSM_BUILD_GUI)
    include(FetchContent)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    FetchContent_MakeAvailable(glfw)
    find_package(OpenGL REQUIRED)

    set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

    add_library(tinyfiledialogs STATIC external/tinyfiledialogs/tinyfiledialogs.c)
    target_include_directories(tinyfiledialogs PUBLIC external/tinyfiledialogs)

    add_executable(xosm_gui src/gui/main.cpp)
    target_link_libraries(xosm_gui PRIVATE xosm_procedural imgui tinyfiledialogs OpenGL::GL)
    set_target_properties(xosm_gui PROPERTIES OUTPUT_NAME xosm-gui)
    if(NOT MSVC)
        target_compile_options(xosm_gui PRIVATE -Wall -Wextra)
    endif()
    if(APPLE)
        target_compile_definitions(xosm_gui PRIVATE GL_SILENCE_DEPRECATION)
    endif()
endif()

if(XOSM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

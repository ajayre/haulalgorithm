cmake_minimum_required(VERSION 3.16)
project(haul_kdtree LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(haul_kdtree SHARED haul_kdtree.cpp)
target_compile_definitions(haul_kdtree PRIVATE HAUL_KDTREE_BUILD)
target_include_directories(haul_kdtree PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanoflann")

if(MSVC)
    target_compile_options(haul_kdtree PRIVATE /O2 /W4)
else()
    target_compile_options(haul_kdtree PRIVATE -O3 -Wall -Wextra)
    target_compile_options(haul_kdtree PRIVATE -fvisibility=hidden)
    set_target_properties(haul_kdtree PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()

set_target_properties(haul_kdtree PROPERTIES
    OUTPUT_NAME "haul_kdtree"
    POSITION_INDEPENDENT_CODE ON
)

install(TARGETS haul_kdtree
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

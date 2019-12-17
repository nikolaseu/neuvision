cmake_minimum_required(VERSION 3.15)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/MunkeiVersionFromGit.cmake)
version_from_git(
    LOG ON
    TIMESTAMP "%Y%m%d%H%M%S"
)

project(NEUVision
    LANGUAGES CXX
    VERSION ${VERSION}
    DESCRIPTION "A framework for structured light scanning projects"
)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(Z3D_VERSION_STR="${VERSION}")
add_compile_definitions(Z3D_VERSION_BUILD_STR="${SEMVER}")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(Z3D_RELEASE_BUILD)
endif()

add_compile_definitions(_USE_MATH_DEFINES)

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# Output directories
set(Z3D_OUTPUT_DIRECTORY ${NEUVision_BINARY_DIR}/bin)
foreach(type LIBRARY RUNTIME ARCHIVE)
    if(NOT DEFINED CMAKE_${type}_OUTPUT_DIRECTORY)
        set(CMAKE_${type}_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY} CACHE INTERNAL "Single output directory for building all libraries.")
    endif()
endforeach()

add_subdirectory(3rdparty/qtpropertybrowser)

add_subdirectory(lib)
add_subdirectory(apps)

cmake_minimum_required(VERSION 3.19)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(DEFINED ENV{CI})
    message(STATUS "Obtaining version from git tag...")
    include(cmake/MunkeiVersionFromGit.cmake)
    version_from_git(
        LOG ON
        TIMESTAMP "%Y%m%d%H%M%S"
    )
else()
    set(VERSION 0.2.0)
    set(SEMVER "${VERSION}-SNAPSHOT")
    message(STATUS "Hardcoded version ${SEMVER}")
endif()

project(NEUVision
    LANGUAGES CXX
    VERSION ${VERSION}
    DESCRIPTION "A framework for structured light scanning projects"
)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

add_compile_definitions(Z3D_VERSION_STR="${VERSION}")
add_compile_definitions(Z3D_VERSION_BUILD_STR="${SEMVER}")
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message("This is a release build (CMAKE_BUILD_TYPE=" ${CMAKE_BUILD_TYPE} ")")
    add_compile_definitions(Z3D_RELEASE_BUILD)
endif()

add_compile_definitions(_USE_MATH_DEFINES)

# disable all the deprecated APIs in Qt <= 6
add_compile_definitions(QT_DISABLE_DEPRECATED_BEFORE=0x060400)

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

enable_testing()
add_subdirectory(tests)

include(FeatureSummary)
feature_summary(WHAT ALL)

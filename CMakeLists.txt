cmake_minimum_required(VERSION 3.15)

project(NEUVision
    LANGUAGES CXX
    VERSION 1.0.0 # FIXME
    DESCRIPTION "A framework for structured light scanning projects"
    )

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(Z3D_VERSION_STR="${CMAKE_PROJECT_VERSION}")
add_compile_definitions(Z3D_VERSION_BUILD_STR="${CMAKE_PROJECT_VERSION}") # FIXME add build number/hash

add_compile_definitions(_USE_MATH_DEFINES)

#-----------------------------------------------------------------------------
# Output directories.
#
set(Z3D_OUTPUT_DIRECTORY ${NEUVision_BINARY_DIR}/bin)
foreach(type LIBRARY RUNTIME ARCHIVE)
    if(NOT DEFINED CMAKE_${type}_OUTPUT_DIRECTORY)
        set(CMAKE_${type}_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY} CACHE INTERNAL "Single output directory for building all libraries.")
    endif()
endforeach()

add_subdirectory(3rdparty/qtpropertybrowser)

add_subdirectory(lib)
add_subdirectory(apps)

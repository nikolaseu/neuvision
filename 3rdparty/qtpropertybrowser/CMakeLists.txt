CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

PROJECT(QtPropertyBrowser)

OPTION(QtPropertyBrowser_BUILD_EXAMPLES "Build examples" OFF)

##-----------------------------------------------------------------------------
## Output directories.
##
#SET(output_dir ${QtPropertyBrowser_BINARY_DIR}/bin)
#FOREACH(type LIBRARY RUNTIME ARCHIVE)
#  IF(NOT DEFINED CMAKE_${type}_OUTPUT_DIRECTORY)
#    SET(CMAKE_${type}_OUTPUT_DIRECTORY ${output_dir} CACHE INTERNAL "Single output directory for building all libraries.")
#  ENDIF()
#ENDFOREACH()

#-----------------------------------------------------------------------------
# Qt
#
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
#INCLUDE(${QT_USE_FILE})

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#-----------------------------------------------------------------------------
# Subdirectories
#
ADD_SUBDIRECTORY(src)
IF(QtPropertyBrowser_BUILD_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
ENDIF()

#-----------------------------------------------------------------------------
# Generate QtPropertyBrowserConfig file
#
CONFIGURE_FILE(UseQtPropertyBrowser.cmake.in
               ${QtPropertyBrowser_BINARY_DIR}/UseQtPropertyBrowser.cmake COPYONLY)

INCLUDE(GenerateQtPropertyBrowserConfig.cmake)

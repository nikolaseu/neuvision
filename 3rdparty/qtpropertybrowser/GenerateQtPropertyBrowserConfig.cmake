###########################################################################
#
#  Library:   CTK
#
#  Copyright (c) Kitware Inc.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.commontk.org/LICENSE
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
###########################################################################

#-----------------------------------------------------------------------------
# Settings shared between the build tree and install tree.


#-----------------------------------------------------------------------------
# Settings specific to the build tree.

# The "use" file.
SET(QtPropertyBrowser_USE_FILE ${QtPropertyBrowser_BINARY_DIR}/UseQtPropertyBrowser.cmake)

# Determine the include directories needed.
SET(QtPropertyBrowser_INCLUDE_DIRS_CONFIG
  ${QtPropertyBrowser_SOURCE_DIR}/src
)

# Library directory.
SET(QtPropertyBrowser_LIBRARY_DIRS_CONFIG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Runtime library directory.
SET(QtPropertyBrowser_RUNTIME_LIBRARY_DIRS_CONFIG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Build configuration information.
SET(QtPropertyBrowser_CONFIGURATION_TYPES_CONFIG ${CMAKE_CONFIGURATION_TYPES})
SET(QtPropertyBrowser_BUILD_TYPE_CONFIG ${CMAKE_BUILD_TYPE})

#-----------------------------------------------------------------------------
# Configure QtPropertyBrowserConfig.cmake for the build tree.
CONFIGURE_FILE(${QtPropertyBrowser_SOURCE_DIR}/QtPropertyBrowserConfig.cmake.in
               ${QtPropertyBrowser_BINARY_DIR}/QtPropertyBrowserConfig.cmake @ONLY IMMEDIATE)

#-----------------------------------------------------------------------------
# Settings specific to the install tree.

# TODO

#-----------------------------------------------------------------------------
# Configure QtPropertyBrowserConfig.cmake for the install tree.

# TODO

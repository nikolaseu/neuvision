find_path(VIMBA_INCLUDE_DIRS
  NAMES
    VimbaC/Include/VimbaC.h
    VimbaCPP/Include/VimbaCPP.h
  PATHS
    $ENV{AVT_VIMBA_DIR}
    /usr/include
    /usr/local/include
)

find_library(VIMBA_LIBRARIES
  NAMES
    VimbaCPP
  PATHS
    $ENV{AVT_VIMBA_DIR}/lib
    $ENV{AVT_VIMBA_DIR}/VimbaCPP/lib
    $ENV{AVT_VIMBA_DIR}/VimbaCPP/lib/win64
    /usr/lib
    /usr/local/lib
)

if(VIMBA_LIBRARIES AND VIMBA_INCLUDE_DIRS)
  set(VIMBA_FOUND TRUE)
else()
  set(VIMBA_FOUND FALSE)
endif()

mark_as_advanced(
  VIMBA_INCLUDE_DIRS
  VIMBA_LIBRARIES
)

add_subdirectory(zqtgeometryloaders)

find_package(PCL COMPONENTS io QUIET)
if(PCL_FOUND)
    add_subdirectory(zpcl)
endif()

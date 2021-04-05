project(ZCore LANGUAGES CXX)

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ZCore/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

file(GLOB QQMLOBJECTLISTMODEL_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel/*.cpp)

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    ${QQMLOBJECTLISTMODEL_SRC_FILES}) # FIXME

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_${PROJECT_NAME}_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Qt::Core)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    RUNTIME
        DESTINATION bin
    LIBRARY
        DESTINATION lib
    ARCHIVE
        DESTINATION lib)

install(EXPORT ${PROJECT_NAME}Config
    FILE ${PROJECT_NAME}Config.cmake
    DESTINATION cmake)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION .)

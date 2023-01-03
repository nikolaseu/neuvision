project(Z3DScanner LANGUAGES CXX)

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Quick QuickControls2 Concurrent REQUIRED)

if(NOT TARGET ZStructuredLight)
    message(WARNING "ZStructuredLight not available, skipping ${PROJECT_NAME}")
    return()
endif()

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(${PROJECT_NAME}
    ${HEADER_FILES}
    ${SRC_FILES}
    resources.qrc
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt::Widgets
        Qt::Concurrent
        Qt::Quick
        Qt::QuickControls2
        ZGui
        ZCameraCalibrator
        ZStructuredLight
        ZPointCloudQml
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE true)
endif()

project(Z3DCloudViewer LANGUAGES CXX)

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Quick QuickControls2 Concurrent Xml REQUIRED)

if(NOT TARGET ZPointCloud)
    message(WARNING "ZPointCloud not available, skipping ${PROJECT_NAME}")
    return()
endif()

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${PROJECT_NAME}
    ${HEADER_FILES}
    ${SRC_FILES}
    qml.qrc
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt::Widgets
        Qt::Quick
        Qt::QuickControls2
        Qt::Concurrent
        Qt::Xml
        ZGui
        ZPointCloud
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE true)
endif()

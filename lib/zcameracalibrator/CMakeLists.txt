project(ZCameraCalibrator LANGUAGES CXX)

find_package(Qt5 COMPONENTS Core Widgets Concurrent REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

file(GLOB UI_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.ui)

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    ${UI_FILES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_CAMERACALIBRATOR_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qtpropertybrowser/src
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
        Qt5::Concurrent
        ${OpenCV_LIBS}
        QtPropertyBrowser
    PUBLIC
        ZCore
        ZGui
        ZCameraAcquisition
        ZCameraCalibration
        ZCalibratedCamera
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

add_subdirectory(finders)

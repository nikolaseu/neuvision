project(ZStereoStructuredLight LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY}/plugins/structuredlight)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY}/plugins/structuredlight)

find_package(Qt5 COMPONENTS Core Concurrent REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

file(GLOB METADATA_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.json)

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    ${METADATA_FILES}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Core
        Qt5::Concurrent
        Qt5::Widgets
        ${OpenCV_LIBS}
        ZCameraCalibration
        ZCameraCalibrator
        ZStructuredLight
)

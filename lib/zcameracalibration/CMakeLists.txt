project(ZCameraCalibration LANGUAGES CXX)

find_package(Qt5 COMPONENTS Core Widgets Concurrent REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB_RECURSE HEADER_FILES
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

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_CAMERACALIBRATION_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
        Qt5::Concurrent
        ${OpenCV_LIBS}
    PUBLIC
        ZCore
        ZGui
)

project(Z3DScanner LANGUAGES CXX)

find_package(Qt5 COMPONENTS Widgets Quick QuickControls2 Concurrent REQUIRED)

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
        Qt5::Widgets
        Qt5::Concurrent
        Qt5::Quick
        Qt5::QuickControls2
    PUBLIC
        ZGui
        ZCameraCalibrator
        ZStructuredLight
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set_property(TARGET ${PROJECT_NAME} PROPERTY WIN32_EXECUTABLE true)
endif()

project(ZStructuredLightPatternBinaryTests LANGUAGES CXX)

if(NOT TARGET ZBinaryPatternProjection)
    message("ZBinaryPatternProjection not available, skipping ${PROJECT_NAME}")
    return()
endif()

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Test REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
    tst_zbinarypatterndecoder.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/zstructuredlight/patterns/binary
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Test
    ZBinaryPatternProjection
    opencv_imgcodecs
)

add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

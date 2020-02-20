project(ZStructuredLightPatternBinaryTests LANGUAGES CXX)

find_package(Qt5 COMPONENTS Test REQUIRED)

add_executable(${PROJECT_NAME}
    tst_zbinarypatterndecoder.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/zstructuredlight/patterns/binary)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Test
    ZBinaryPatternProjection)

add_test(NAME ${PROJECT_NAME} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

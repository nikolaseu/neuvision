project(Z3DCloudViewer LANGUAGES CXX)

find_package(Qt5 COMPONENTS Widgets Quick Concurrent Xml REQUIRED)

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
        Qt5::Widgets
        Qt5::Quick
        Qt5::Concurrent
        Qt5::Xml
    PUBLIC
        ZGui
        ZPointCloud
)

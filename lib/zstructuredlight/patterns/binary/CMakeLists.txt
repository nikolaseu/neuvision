project(ZBinaryPatternProjection LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY}/plugins/structuredlightpatterns)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${Z3D_OUTPUT_DIRECTORY}/plugins/structuredlightpatterns)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # it's a plugin, just export everything, makes it easier for tests

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick REQUIRED)
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
    zbinarypatternprojection_resources.qrc
)

set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

target_link_libraries(${PROJECT_NAME}
    PUBLIC # it's a plugin, so public is fine, and makes it easier for tests (no need to specify all dependencies)
        Qt::Core
        Qt::Quick
        opencv_core
        ZStructuredLight
)

project(ZPointCloud LANGUAGES CXX)

find_package(Qt5 COMPONENTS Core Qml 3DCore 3DInput 3DQuick 3DRender 3DQuickRender 3DLogic REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

file(GLOB QQMLOBJECTLISTMODEL_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel/*.cpp)

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    src/zpointcloud_resources.qrc
    ${QQMLOBJECTLISTMODEL_SRC_FILES} # FIXME
)

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_${PROJECT_NAME}_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Qml
        Qt5::3DCore
        Qt5::3DInput
        Qt5::3DQuick
        Qt5::3DRender
        Qt5::3DQuickRender
        Qt5::3DLogic
    PUBLIC
        ZCore
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

add_subdirectory(plugins)

project(ZGui LANGUAGES CXX)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

if(APPLE)
    list(APPEND SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/zosxutils.mm)
endif()

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_GUI_LIBRARY)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
    PUBLIC
        ZCore
)

if(APPLE)
    # for macOS
    target_link_libraries(${PROJECT_NAME} PRIVATE "-framework Cocoa")
endif()

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
    ${SRC_FILES})

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_${PROJECT_NAME}_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
    PUBLIC
        ZCore)

if(APPLE)
    # for macOS
    target_link_libraries(${PROJECT_NAME} PRIVATE "-framework Cocoa")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    RUNTIME
        DESTINATION bin
    LIBRARY
        DESTINATION lib
    ARCHIVE
        DESTINATION lib)

install(EXPORT ${PROJECT_NAME}Config
    FILE ${PROJECT_NAME}Config.cmake
    DESTINATION cmake)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION .)

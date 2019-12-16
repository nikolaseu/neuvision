project(ZCore LANGUAGES CXX)

find_package(Qt5 COMPONENTS Core REQUIRED)

file(GLOB HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ZCore/*.h)

file(GLOB SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

file(GLOB QQMLOBJECTLISTMODEL_SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel/*.cpp)

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    ${QQMLOBJECTLISTMODEL_SRC_FILES} # FIXME
)

target_compile_definitions(${PROJECT_NAME} PRIVATE Z3D_CORE_LIBRARY)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/qqmlobjectlistmodel
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Core)

#set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/ZCore/*.h)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${CMAKE_PROJECT_VERSION})
#set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${CMAKE_PROJECT_VERSION})

#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(FILES ${HEADER_FILES} DESTINATION include/${PROJECT_NAME})

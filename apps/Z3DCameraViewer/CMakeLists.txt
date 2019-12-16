project(Z3DCameraViewer LANGUAGES CXX)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
    PUBLIC
        ZCameraAcquisition
)

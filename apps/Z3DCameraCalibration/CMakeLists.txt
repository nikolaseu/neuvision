project(Z3DCameraCalibration LANGUAGES CXX)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt5::Widgets
    PUBLIC
        ZCameraAcquisition
        ZCameraCalibration
        ZCameraCalibrator
)

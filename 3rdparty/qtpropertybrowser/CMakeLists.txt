cmake_minimum_required(VERSION 3.12)

project(QtPropertyBrowser
    LANGUAGES CXX
    )

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Widgets REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB HEADER_FILES
    *.h
    )

file(GLOB SRC_FILES
    *.cpp
    )

add_library(${PROJECT_NAME}
    ${HEADER_FILES}
    ${SRC_FILES}
    )

target_link_libraries(${PROJECT_NAME}
    Qt::Core
    Qt::Widgets
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    )

project(ZPointCloudQml LANGUAGES CXX)

find_package(QT NAMES Qt5 Qt6 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Qml 3DCore 3DInput 3DQuick 3DRender 3DQuickRender 3DLogic)

if(NOT TARGET Qt::3DCore)
    message(WARNING "Qt3D not available, skipping ${PROJECT_NAME}")
    return()
endif()

qt_add_qml_module(ZPointCloudQml
    URI Z3D.ZPointCloud
    VERSION 1.0
    RESOURCE_PREFIX z3d.neuvision
    SOURCES
        zpointcloud_qml.cpp
        zpointcloud_qml.h
        zpointcloudgeometry.cpp
        zpointcloudgeometry.h
        zpointcloudreader.cpp
        zpointcloudreader.h
    QML_FILES
        BasicCamera.qml
        DefaultCameraController.qml
        PointCloudEntity.qml
    RESOURCES
        shaders/pointcloud.frag
        shaders/pointcloud.geom
        shaders/pointcloud.vert
        shaders/pointcloud_basic.frag
        shaders/pointcloud_basic.vert
    IMPORTS
        Qt3D.Core
        Qt3D.Extras
        Qt3D.Input
        Qt3D.Logic
        Qt3D.Render
        QtQuick
    #ENABLE_TYPE_COMPILER # Qt3D not compiling correctly yet?
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt::Qml
        Qt::3DCore
        Qt::3DInput
        Qt::3DQuick
        Qt::3DRender
        Qt::3DQuickRender
        Qt::3DLogic
    PUBLIC
        ZCore
        ZPointCloud
)

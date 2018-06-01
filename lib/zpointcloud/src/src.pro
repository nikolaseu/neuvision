include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl #gui #
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY

HEADERS      += \
    Z3DPointCloud \
    zcloudviewwindow.h \
    zpointcloud.h \
    zpointcloud_fwd.h \
    zpointcloud_global.h \
    zpointcloud_typedefs.h \

SOURCES      += \
    zcloudviewwindow.cpp \
    zpointcloud.cpp \

FORMS        += \
    zcloudviewwindow.ui

###############################################################################
# VTK Qt support without having to build VTK explicitly with Qt support
INCLUDEPATH += $$PWD/../../../3rdparty/VTK/GUISupport/Qt
HEADERS      += \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKInteractor.h \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKInteractorAdapter.h \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKInteractorInternal.h \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKOpenGLWidget.h \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKOpenGLWindow.h \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKWin32Header.h \
    ../../../3rdparty/VTK/GUISupport/Qt/vtkEventQtSlotConnect.h \
    ../../../3rdparty/VTK/GUISupport/Qt/vtkGUISupportQtModule.h \
    ../../../3rdparty/VTK/GUISupport/Qt/vtkQtConnection.h \

SOURCES      += \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKInteractor.cxx \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKInteractorAdapter.cxx \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKOpenGLWidget.cxx \
    ../../../3rdparty/VTK/GUISupport/Qt/QVTKOpenGLWindow.cxx \
    ../../../3rdparty/VTK/GUISupport/Qt/vtkEventQtSlotConnect.cxx \
    ../../../3rdparty/VTK/GUISupport/Qt/vtkQtConnection.cxx \

###############################################################################
# Camera acquisition
include($$PWD/../../zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../../zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../../zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# VTK
include($$PWD/../../../3rdparty/vtk.pri)

###############################################################################
# Point Cloud Library
include($$PWD/../../../3rdparty/pcl.pri)

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

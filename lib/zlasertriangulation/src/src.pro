include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
#QT           -= gui // we use QImage, QGraphicsView, etc
QT           += opengl # QGLWidget for ZScanImageViewer
greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
}
TARGET        = $$qtLibraryTarget(zlasertriangulation)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_LASERTRIANGULATION_LIBRARY
HEADERS      += \
    zscanimageviewer.h \
    zpointcloudrecorder.h \
    zltsprovider.h \
    zltsplanecalibration.h \
    zltscamerapreviewer.h \
    zltscamera3dcalibrationinterface.h \
    zltscamera3d.h \
    zlasertriangulation_global.h \
    zcamera3dinterface.h \
    zltsprofileplot.h \
    zscanimageviewerfilter.h
SOURCES      += \
    zscanimageviewer.cpp \
    zltscamera3d.cpp \
    zpointcloudrecorder.cpp \
    zltsprovider.cpp \
    zltsplanecalibration.cpp \
    zltscamerapreviewer.cpp \
    zltsprofileplot.cpp \
    zscanimageviewerfilter.cpp
FORMS += \
    zltscamerapreviewer.ui

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Point Cloud Library
include($$PWD/../../../3rdparty/pcl.pri)

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
# Points clouds
include($$PWD/../../zpointcloud/zpointcloud.pri)

###############################################################################
# Qwt
include($$PWD/../../../3rdparty/qwt.pri)

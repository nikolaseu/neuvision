include(../NEUVision.pri)

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Z3DCloudViewer
TEMPLATE = app

#CONFIG += console

DESTDIR = $$Z3D_BUILD_DIR

SOURCES += \
    main.cpp \

###############################################################################
# Core
include($$PWD/../lib/zcore/zcore.pri)

###############################################################################
# Camera acquisition
include($$PWD/../lib/zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../lib/zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../lib/zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# Points clouds
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Point Cloud Library
include($$PWD/../3rdparty/pcl.pri)

###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../3rdparty/qtpropertybrowser/src/qtpropertybrowser.pri)

###############################################################################
# Qwt
include($$PWD/../3rdparty/qwt.pri)

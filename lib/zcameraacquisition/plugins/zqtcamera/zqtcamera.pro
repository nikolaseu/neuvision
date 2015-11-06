include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
QT           += multimedia
TARGET        = $$qtLibraryTarget(zqtcameraplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS      += \
    zqtcamera.h \
    zqtcameraplugin.h
SOURCES      += \
    zqtcamera.cpp \
    zqtcameraplugin.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
QT           += multimedia multimediawidgets
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
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

DISTFILES += \
    zqtcamera.json

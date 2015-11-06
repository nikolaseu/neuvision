include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zflycapture2plugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zflycapture2camera.h \
    zflycapture2plugin.h
SOURCES       = \
    zflycapture2camera.cpp \
    zflycapture2plugin.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

###############################################################################
# FlyCapture
include(3rdparty/FlyCapture2/flycapture2.pri)

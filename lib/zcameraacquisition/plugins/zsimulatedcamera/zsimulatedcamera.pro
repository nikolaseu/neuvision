include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zsimulatedcameraplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zsimulatedcameraplugin.h \
    zsimulatedcamera.h
SOURCES       = \
    zsimulatedcameraplugin.cpp \
    zsimulatedcamera.cpp
OTHER_FILES += \
    zsimulatedcamera.json

###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera acquisition
include($$PWD/../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

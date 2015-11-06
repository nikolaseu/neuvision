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
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

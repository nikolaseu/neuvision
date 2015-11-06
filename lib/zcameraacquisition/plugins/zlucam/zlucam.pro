include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zlucamplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zlucamplugin.h \
    zlucamcamera.h
SOURCES       = \
    zlucamcamera.cpp \
    zlucamplugin.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# Lumenera LuCam
include(3rdparty/LuCam/lucam.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

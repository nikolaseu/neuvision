include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zpleoraebusplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zpleoraebuscamera.h \
    zpleoraebusplugin.h \
    zpleoraebuscamera_p.h \
    at_c4_2040_chunkmodedefs.h
SOURCES       = \
    zpleoraebuscamera.cpp \
    zpleoraebusplugin.cpp \
    zpleoraebuscamera_p.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# Pleora eBus SDK
include(3rdparty/pleoraebussdk.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

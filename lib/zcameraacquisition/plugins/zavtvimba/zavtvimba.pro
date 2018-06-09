include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zavtvimbaplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zavtvimbacamera.h \
    zavtvimbaplugin.h
SOURCES       = \
    zavtvimbacamera.cpp \
    zavtvimbaplugin.cpp
DISTFILES += \
    README.md \
    zavtvimba.json \

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# Allied Vision Technologies Vimba SDK
include(3rdparty/VimbaCPP.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

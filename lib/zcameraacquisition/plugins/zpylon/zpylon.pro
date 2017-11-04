include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zpylonplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zpylonplugin.h \
    zpyloncamera.h
SOURCES       = \
    zpyloncamera.cpp \
    zpylonplugin.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# Basler Pylon
include(3rdparty/pylon.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

DISTFILES += \
    zpylon.json

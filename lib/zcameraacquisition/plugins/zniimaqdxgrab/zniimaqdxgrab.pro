include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zniimaqdxgrabplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
HEADERS       = \
    zniimaqdxgrabcamera.h \
    zniimaqdxgrabplugin.h \
    zniimaqdxgrabcamera_p.h
SOURCES       = \
    zniimaqdxgrabcamera.cpp \
    zniimaqdxgrabplugin.cpp \
    zniimaqdxgrabcamera_p.cpp

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# National Instruments NI IMAQdx
include(3rdparty/niimaqdx.pri)

###############################################################################
# National Instruments NI Vision
include(3rdparty/nivision.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

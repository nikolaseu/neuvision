include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
TARGET        = $$qtLibraryTarget(zopencvvideocaptureplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zopencvvideocapturecamera.h \
    zopencvvideocaptureplugin.h
SOURCES       = \
    zopencvvideocapturecamera.cpp \
    zopencvvideocaptureplugin.cpp
OTHER_FILES += \
    zopencvvideocapture.json

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

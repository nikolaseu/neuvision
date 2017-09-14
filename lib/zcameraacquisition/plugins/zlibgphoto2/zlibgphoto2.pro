include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui // I nees gui because I use QImage to convert jpg images
TARGET        = $$qtLibraryTarget(zlibgphoto2plugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/cameraacquisition
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zlibgphoto2camera.h \
    zlibgphoto2plugin.h
SOURCES       = \
    zlibgphoto2camera.cpp \
    zlibgphoto2plugin.cpp
OTHER_FILES += \
    zlibgphoto2.json

###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera acquisition
include(../../zcameraacquisition.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

###############################################################################
# libgphoto2
include(3rdparty/libgphoto2.pri)

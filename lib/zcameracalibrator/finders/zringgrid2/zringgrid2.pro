include(../../../../NEUVision.pri)

VERSION = $$Z3D_VERSION
TEMPLATE = lib
CONFIG += plugin
QT += widgets
TARGET = $$qtLibraryTarget(zringgrid2calibrationpatternfinderplugin)
DESTDIR = $$Z3D_BUILD_DIR/plugins/calibrationpatternfinder

HEADERS = \
    zringgrid2patternfinder.h \
    zringgrid2patternfinderconfigwidget.h \
    zringgrid2patternfinderplugin.h

SOURCES = \
    zringgrid2patternfinder.cpp \
    zringgrid2patternfinderconfigwidget.cpp \
    zringgrid2patternfinderplugin.cpp

FORMS += \
    zringgrid2patternfinderconfigwidget.ui

win32:DEFINES += _USE_MATH_DEFINES # for M_PI

###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

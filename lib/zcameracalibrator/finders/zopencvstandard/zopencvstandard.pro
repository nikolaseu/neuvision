include(../../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui
QT           += widgets
TARGET        = $$qtLibraryTarget(zopencvstandardpatternfinderplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/calibrationpatternfinder

HEADERS       = \
    zopencvstandardpatternfinderplugin.h \
    zchessboardcalibrationpatternfinder.h \
    zchessboardcalibrationpatternfinderconfigwidget.h \
    zcirclegridcalibrationpatternfinder.h \
    zcirclegridcalibrationpatternfinderconfigwidget.h
    
SOURCES       = \
    zopencvstandardpatternfinderplugin.cpp \
    zchessboardcalibrationpatternfinder.cpp \
    zchessboardcalibrationpatternfinderconfigwidget.cpp \
    zcirclegridcalibrationpatternfinder.cpp \
    zcirclegridcalibrationpatternfinderconfigwidget.cpp

FORMS += \
    zchessboardcalibrationpatternfinderconfigwidget.ui \
    zcirclegridcalibrationpatternfinderconfigwidget.ui

###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

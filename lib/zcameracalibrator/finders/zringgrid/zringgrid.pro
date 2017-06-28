include(../../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui
greaterThan(QT_MAJOR_VERSION, 4) {
    QT       += widgets
}
TARGET        = $$qtLibraryTarget(zringgridcalibrationpatternfinderplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/calibrationpatternfinder

HEADERS       = \
    zringgridpatternfinder.h \
    zringgridpatternfinderconfigwidget.h \
    zringgridpatternfinderplugin.h
    
SOURCES       = \
    zringgridpatternfinder.cpp \
    zringgridpatternfinderconfigwidget.cpp \
    zringgridpatternfinderplugin.cpp

FORMS += \
    zringgridpatternfinderconfigwidget.ui

###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

include(../../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui
greaterThan(QT_MAJOR_VERSION, 4) {
    QT       += widgets
}
TARGET        = $$qtLibraryTarget(zincompletecirclegridcalibrationpatternfinderplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/calibrationpatternfinder

HEADERS       = \
    zincompletecirclegridpatternfinder.h \
    zincompletecirclegridpatternfinderconfigwidget.h \
    zincompletecirclegridpatternfinderplugin.h
    
SOURCES       = \
    zincompletecirclegridpatternfinder.cpp \
    zincompletecirclegridpatternfinderconfigwidget.cpp \
    zincompletecirclegridpatternfinderplugin.cpp

FORMS += \
    zincompletecirclegridpatternfinderconfigwidget.ui

###############################################################################
# Camera calibrator
include(../../zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

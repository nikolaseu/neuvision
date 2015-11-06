include(../../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui
greaterThan(QT_MAJOR_VERSION, 4) {
    QT       += widgets
}
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
# Camera calibrator
include(../../zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

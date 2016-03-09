include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
greaterThan(QT_MAJOR_VERSION, 4) {
    QT -= gui
    QT += widgets concurrent
}
TARGET        = $$qtLibraryTarget(zslstereoplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/structuredlight
VERSION       = $$Z3D_VERSION
HEADERS       = \
    zstereosls.h \
    zstereoslsconfigwidget.h \
    zstereosystemimpl.h \
    zstereoslsplugin.h

SOURCES       = \
    zstereosls.cpp \
    zstereoslsconfigwidget.cpp \
    zstereosystemimpl.cpp \
    zstereoslsplugin.cpp

FORMS        += \
    zstereoslsconfigwidget.ui



###############################################################################
# Core
include($$PWD/../../../zcore/zcore.pri)

###############################################################################
# Structured light system
include($$PWD/../../zstructuredlight.pri)

###############################################################################
# Camera acquisition
include($$PWD/../../../zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibrations
include($$PWD/../../../zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../../../zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../../zcameracalibrator/zcameracalibrator.pri)

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

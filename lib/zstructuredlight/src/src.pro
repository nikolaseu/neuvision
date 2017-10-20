include(../../../NEUVision.pri)

TEMPLATE      = lib
greaterThan(QT_MAJOR_VERSION, 4) {
    QT -= gui
    QT += widgets quick concurrent
}
TARGET        = $$qtLibraryTarget(zstructuredlight)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_STRUCTUREDLIGHT_LIBRARY
SOURCES += \
    zpatternprojectionprovider.cpp \
    zcameraacquisitionmanager.cpp \
    zdecodedpattern.cpp \
    zgeometryutils.cpp \
    zpatternprojection.cpp \
    zsimplepointcloud.cpp \
    zstructuredlightsystem.cpp \
    zstructuredlightsystemprovider.cpp \
    zprojectedpattern.cpp \
    zstructuredlightpattern.cpp

HEADERS += \
    zpatternprojectionprovider.h \
    zstructuredlight_global.h \
    zcameraacquisitionmanager.h \
    zdecodedpattern.h \
    zgeometryutils.h \
    zpatternprojection.h \
    zsimplepointcloud.h \
    zstructuredlightsystem.h \
    zpatternprojectionplugin.h \
    zstructuredlightsystemplugin.h \
    zstructuredlightsystemprovider.h \
    zprojectedpattern.h \
    zstructuredlightpattern.h

RESOURCES += \
    resources.qrc



###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

###############################################################################
# Camera acquisition
include($$PWD/../../zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../../zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../../zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator/zcameracalibrator.pri)

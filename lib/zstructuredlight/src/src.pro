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
    zbinarypatterndecoder.cpp \
    zbinarypatternprojection.cpp \
    zbinarypatternprojectionconfigwidget.cpp \
    zcameraacquisitionmanager.cpp \
    zdecodedpattern.cpp \
    zgeometryutils.cpp \
    zpatternprojection.cpp \
    zsimplepointcloud.cpp \
    zstereosls.cpp \
    zstereoslsconfigwidget.cpp \
    zstereosystemimpl.cpp \
    zstructuredlightsystem.cpp \
    zthreephase.cpp

HEADERS += \
    zpatternprojectionprovider.h \
    zstructuredlight_global.h \
    zbinarypatterndecoder.h \
    zbinarypatternprojection.h \
    zbinarypatternprojectionconfigwidget.h \
    zcameraacquisitionmanager.h \
    zdecodedpattern.h \
    zgeometryutils.h \
    zpatternprojection.h \
    zsimplepointcloud.h \
    zstereosls.h \
    zstereoslsconfigwidget.h \
    zstereosystemimpl.h \
    zstructuredlightsystem.h \
    zthreephase.h

FORMS += \
    zbinarypatternprojectionconfigwidget.ui \
    zstereoslsconfigwidget.ui

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
# Points clouds
include($$PWD/../../zpointcloud/zpointcloud.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator/zcameracalibrator.pri)

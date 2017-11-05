include(../../../NEUVision.pri)

TEMPLATE      = lib
QT           -= gui
QT           += widgets quick concurrent
TARGET        = $$qtLibraryTarget(zstructuredlight)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_STRUCTUREDLIGHT_LIBRARY

HEADERS += \
    Z3DStructuredLight \
    zcameraacquisitionmanager.h \
    zdecodedpattern.h \
    zgeometryutils.h \
    zpatternprojection.h \
    zpatternprojectionplugin.h \
    zpatternprojectionprovider.h \
    zprojectedpattern.h \
    zsimplepointcloud.h \
    zstructuredlight_fwd.h \
    zstructuredlight_global.h \
    zstructuredlightpattern.h \
    zstructuredlightsystem.h \
    zstructuredlightsystemplugin.h \
    zstructuredlightsystemprovider.h \

SOURCES += \
    zcameraacquisitionmanager.cpp \
    zdecodedpattern.cpp \
    zgeometryutils.cpp \
    zpatternprojection.cpp \
    zpatternprojectionprovider.cpp \
    zprojectedpattern.cpp \
    zsimplepointcloud.cpp \
    zstructuredlightpattern.cpp \
    zstructuredlightsystem.cpp \
    zstructuredlightsystemprovider.cpp \

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

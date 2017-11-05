include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
QT           += widgets concurrent
TARGET        = $$qtLibraryTarget(zslstereoplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/structuredlight
VERSION       = $$Z3D_VERSION

HEADERS       = \
    zdualcamerastereosls.h \
    zdualcamerastereoslsconfigwidget.h \
    zsinglecamerastereosls.h \
    zstereosls.h \
    zstereoslsplugin.h \
    zstereosystemimpl.h \

SOURCES       = \
    zdualcamerastereosls.cpp \
    zdualcamerastereoslsconfigwidget.cpp \
    zsinglecamerastereosls.cpp \
    zstereosls.cpp \
    zstereoslsplugin.cpp \
    zstereosystemimpl.cpp \

FORMS        += \
    zdualcamerastereoslsconfigwidget.ui



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

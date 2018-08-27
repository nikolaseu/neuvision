include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
QT           -= gui
QT           += widgets quick
TARGET        = $$qtLibraryTarget(zbinaryprojectionplugin)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/structuredlightpatterns
VERSION       = $$Z3D_VERSION

HEADERS       = \
    zbinarypatterndecoder.h \
    zbinarypatternprojection.h \
    zbinarypatternprojectionplugin.h \

SOURCES       = \
    zbinarypatterndecoder.cpp \
    zbinarypatternprojection.cpp \
    zbinarypatternprojectionplugin.cpp \

RESOURCES    += \
    resources.qrc



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
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)


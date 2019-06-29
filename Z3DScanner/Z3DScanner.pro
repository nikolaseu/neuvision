include(../NEUVision.pri)

QT += core gui widgets quick quickcontrols2 qml concurrent 3dinput 3dquick 3drender 3dquickrender
DESTDIR = $$Z3D_BUILD_DIR
TARGET = Z3DScanner
VERSION = $$Z3D_VERSION
TEMPLATE = app

# Define this to limit QtConcurrent thread pool size
#DEFINES += Z3D_THREAD_COUNT_LIMIT=1

###############################################################################
# Project
SOURCES += \
    src/main.cpp \
    src/zscannerqml.cpp \

HEADERS += \
    src/zscannerqml.h \

RESOURCES += \
    resources.qrc



###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../3rdparty/qtpropertybrowser/src/qtpropertybrowser.pri)

###############################################################################
# Core
include($$PWD/../lib/zcore/zcore.pri)

###############################################################################
# Gui
include($$PWD/../lib/zgui/zgui.pri)

###############################################################################
# Camera acquisition
include($$PWD/../lib/zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../lib/zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../lib/zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# Structured Light
include($$PWD/../lib/zstructuredlight/zstructuredlight.pri)

###############################################################################
# Point cloud
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

###############################################################################
# Android specific
android:{
    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/../android-sources
    ANDROID_EXTRA_PLUGINS *= $$Z3D_BUILD_DIR/plugins
}

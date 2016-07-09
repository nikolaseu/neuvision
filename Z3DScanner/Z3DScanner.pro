include(../NEUVision.pri)

TEMPLATE = app

TARGET = Z3DScanner

QT += core gui opengl multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets quick qml concurrent

#CONFIG += console

DESTDIR = $$Z3D_BUILD_DIR

win32:DEFINES += _USE_MATH_DEFINES # pcl-1.7 lo define, pero 1.6 no

# Define this to limit QtConcurrent thread pool size
#DEFINES += Z3D_THREAD_COUNT_LIMIT=1

###############################################################################
# Project
SOURCES += \
    src/main.cpp \
    src/ui/mainwindow.cpp \
    src/zscannerinitialconfigwizard.cpp \
    src/zpointcloudwidget.cpp \
    src/zpointclouddata.cpp

HEADERS += \
    src/ui/mainwindow.h \
    src/zscannerinitialconfigwizard.h \
    src/zpointcloudwidget.h \
    src/zpointclouddata.h

FORMS += \
    src/ui/mainwindow.ui \
    src/zscannerinitialconfigwizard.ui

RESOURCES += \
    resources.qrc



###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../3rdparty/qtpropertybrowser/src/qtpropertybrowser.pri)

###############################################################################
# Qwt
include($$PWD/../3rdparty/qwt.pri)

###############################################################################
# Core
include($$PWD/../lib/zcore/zcore.pri)

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
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

###############################################################################
# Android specific
android:{
    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/../android-sources
    ANDROID_EXTRA_PLUGINS *= $$Z3D_BUILD_DIR/plugins
}

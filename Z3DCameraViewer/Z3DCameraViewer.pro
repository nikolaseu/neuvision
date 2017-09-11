include(../NEUVision.pri)

QT       += core gui multimedia widgets

#CONFIG += console

DESTDIR = $$Z3D_BUILD_DIR

TARGET = Z3DCameraViewer

TEMPLATE = app

###############################################################################
# Project files
SOURCES  += \
    main.cpp

HEADERS  +=

FORMS    +=

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
# Camera acquisition
include($$PWD/../lib/zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Android specific
android:{
    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/../android-sources
    ANDROID_EXTRA_PLUGINS *= $$Z3D_BUILD_DIR/plugins
}

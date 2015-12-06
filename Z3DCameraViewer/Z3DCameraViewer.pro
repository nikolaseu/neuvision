include(../NEUVision.pri)

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
}

CONFIG += c++11
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

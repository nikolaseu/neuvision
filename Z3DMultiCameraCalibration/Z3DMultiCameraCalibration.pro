include(../NEUVision.pri)

QT       += core gui widgets
#CONFIG += console
DESTDIR = $$Z3D_BUILD_DIR
TARGET = Z3DMultiCameraCalibration
VERSION = $$Z3D_VERSION
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
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

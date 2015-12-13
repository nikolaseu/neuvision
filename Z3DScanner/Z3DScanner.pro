include(../NEUVision.pri)

TEMPLATE = app

TARGET = Z3DScanner

QT += core gui opengl #declarative

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets quick qml concurrent

CONFIG += c++11
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
    src/sls/zcameraacquisitionmanager.cpp \
    src/sls/zpatternprojection.cpp \
    src/sls/zstructuredlightsystem.cpp \
    src/sls/zsimplepointcloud.cpp \
    src/sls/zbinarypatternprojection.cpp \
    src/sls/zbinarypatternprojectionconfigwidget.cpp \
    src/sls/zdecodedpattern.cpp \
    src/sls/zgeometryutils.cpp \
    src/sls/zthreephase.cpp \
    src/sls/zbinarypatterndecoder.cpp \
    src/sls/zstereoslsconfigwidget.cpp \
    src/sls/zstereosls.cpp \
    src/sls/zstereosystemimpl.cpp

HEADERS += \
    src/ui/mainwindow.h \
    src/zscannerinitialconfigwizard.h \
    src/sls/zcameraacquisitionmanager.h \
    src/sls/zpatternprojection.h \
    src/sls/zstructuredlightsystem.h \
    src/sls/zsimplepointcloud.h \
    src/sls/zbinarypatternprojection.h \
    src/sls/zbinarypatternprojectionconfigwidget.h \
    src/sls/zdecodedpattern.h \
    src/sls/zgeometryutils.h \
    src/sls/zthreephase.h \
    src/sls/zbinarypatterndecoder.h \
    src/sls/zstereoslsconfigwidget.h \
    src/sls/zstereosls.h \
    src/sls/zstereosystemimpl.h

FORMS += \
    src/ui/mainwindow.ui \
    src/zscannerinitialconfigwizard.ui \
    src/sls/zbinarypatternprojectionconfigwidget.ui \
    src/sls/zstereoslsconfigwidget.ui

RESOURCES += \
    resources.qrc



###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

###############################################################################
# PCL, VTK, etc etc
#include($$PWD/../3rdparty/pcl.pri)

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
# Points clouds
#include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

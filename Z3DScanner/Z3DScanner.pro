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
    src/sls/geometryutils.cpp \
    src/sls/stereosystem.cpp \
    src/sls/threephase.cpp \
    src/sls/binarypatternprojection.cpp \
    src/sls/decodedpattern.cpp \
    src/sls/binarypatternprojectionconfigwidget.cpp \
    zscannerinitialconfigwizard.cpp

HEADERS += \
    src/ui/mainwindow.h \
    src/sls/geometryutils.h \
    src/sls/stereosystem.h \
    src/sls/threephase.h \
    src/sls/binarypatternprojection.h \
    src/sls/decodedpattern.h \
    src/sls/binarypatternprojectionconfigwidget.h \
    zscannerinitialconfigwizard.h

FORMS += \
    src/ui/mainwindow.ui \
    src/sls/binarypatternprojectionconfigwidget.ui \
    zscannerinitialconfigwizard.ui

RESOURCES += \
    resources.qrc



###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

###############################################################################
# PCL, VTK, etc etc
include($$PWD/../3rdparty/pcl.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../3rdparty/qtpropertybrowser/src/qtpropertybrowser.pri)

###############################################################################
# Qwt
include($$PWD/../3rdparty/qwt.pri)

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
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

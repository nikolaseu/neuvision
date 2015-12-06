include(../NEUVision.pri)

QT       += core gui opengl concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LTSAcquisition
TEMPLATE = app

DESTDIR = $$Z3D_BUILD_DIR

# source code
HEADERS  += \
    configurationhelperwindow.h \
    mainwindow.h \
    pointcloudwindow.h \
    zltsthreadedcamera.h
SOURCES  += \
    configurationhelperwindow.cpp \
    main.cpp \
    mainwindow.cpp \
    pointcloudwindow.cpp \
    zltsthreadedcamera.cpp
FORMS    += \
    calibrationwindow.ui \
    configurationhelperwindow.ui \
    mainwindow.ui \
    pointcloudwindow.ui


###############################################################################
# OpenCV
include($$PWD/../3rdparty/opencv.pri)

################################################################################
## PCL, VTK, etc etc
include($$PWD/../3rdparty/pcl.pri)

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
# Camera calibrator
include($$PWD/../lib/zcameracalibrator/zcameracalibrator.pri)

###############################################################################
# Points clouds
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Laser Triangulation
include($$PWD/../lib/zlasertriangulation/zlasertriangulation.pri)

###############################################################################
# Laser Triangulation calibration
include($$PWD/../lib/zlasertriangulationcalibrator/zlasertriangulationcalibrator.pri)

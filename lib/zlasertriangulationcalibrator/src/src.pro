include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
#QT           -= gui
QT           += opengl concurrent
TARGET        = $$qtLibraryTarget(zlasertriangulationcalibrator)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_LASERTRIANGULATIONCALIBRATOR_LIBRARY
HEADERS      += \
    zlasertriangulationcalibrator.h \
    zlasertriangulationcalibrator_global.h \
    zltscalibrationimage.h \
    zlasertriangulationcalibratorwindow.h \
    zltscalibrationimagemodel.h
SOURCES      += \
    zlasertriangulationcalibrator.cpp \
    zltscalibrationimage.cpp \
    zlasertriangulationcalibratorwindow.cpp \
    zltscalibrationimagemodel.cpp

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Point Cloud Library
include($$PWD/../../../3rdparty/pcl.pri)

###############################################################################
# Camera acquisition
include($$PWD/../../zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../../zcameracalibration/zcameracalibration.pri)

###############################################################################
# Calibrated camera
include($$PWD/../../zcalibratedcamera/zcalibratedcamera.pri)

###############################################################################
# Camera calibrator
include($$PWD/../../zcameracalibrator/zcameracalibrator.pri)

###############################################################################
# Points clouds
include($$PWD/../../zpointcloud/zpointcloud.pri)

FORMS += \
    zlasertriangulationcalibratorwindow.ui

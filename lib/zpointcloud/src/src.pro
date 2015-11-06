include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl #gui #
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY
HEADERS      += \
    zcloudview.h \
    zpointcloud.h \
    zpointcloud_typedefs.h \
    zpointcloud_global.h
SOURCES      += \
    zcloudview.cpp \
    zpointcloud.cpp
FORMS        += \
    zcloudview.ui

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
# Point Cloud Library
include($$PWD/../../../3rdparty/pcl.pri)

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

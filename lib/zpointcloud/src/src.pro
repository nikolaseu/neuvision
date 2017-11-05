include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl #gui #
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY

HEADERS      += \
    Z3DPointCloud \
    zcloudviewwindow.h \
    zpointcloud.h \
    zpointcloud_fwd.h \
    zpointcloud_global.h \
    zpointcloud_typedefs.h \

SOURCES      += \
    zcloudviewwindow.cpp \
    zpointcloud.cpp \

FORMS        += \
    zcloudviewwindow.ui



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

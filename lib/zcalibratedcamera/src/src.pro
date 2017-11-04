include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
QT           -= gui
TARGET        = $$qtLibraryTarget(zcalibratedcamera)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_CALIBRATEDCAMERA_LIBRARY

HEADERS      += \
    Z3DCalibratedCamera \
    zcalibratedcamera.h \
    zcalibratedcamera_fwd.h \
    zcalibratedcamera_global.h \
    zcalibratedcameraprovider.h \

SOURCES      += \
    zcalibratedcamera.cpp \
    zcalibratedcameraprovider.cpp \



###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Camera acquisition
include($$PWD/../../zcameraacquisition/zcameraacquisition.pri)

###############################################################################
# Camera calibration
include($$PWD/../../zcameracalibration/zcameracalibration.pri)

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

mac {
# to use the fix for QVTK widget and retina displays (this works at least for VTK7.0)
# http://public.kitware.com/pipermail/vtkusers/2015-February/090117.html
LIBS += -framework Foundation
HEADERS += osx/osxhelper.h
OBJECTIVE_SOURCES += osx/osxhelper.mm
}

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

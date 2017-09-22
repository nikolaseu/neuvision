include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
# CONFIG       += plugin
#QT           -= gui // mientras use QPixmap necesito gui
QT           += widgets concurrent
TARGET        = $$qtLibraryTarget(zcameracalibrator)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_CAMERACALIBRATOR_LIBRARY
HEADERS      += \
    zcameracalibrator_global.h \
    zcalibrationimage.h \
    zcalibrationimagemodel.h \
    zcalibrationpatternfinder.h \
    zcameracalibratorworker.h \
    zcalibrationimageviewer.h \
    zmulticalibrationimage.h \
    zmulticalibrationimagemodel.h \
    zmulticameracalibratorworker.h \
    3rdParty/objectcontroller.h \
    zcalibrationpatternfinderplugininterface.h \
    zcalibrationpatternfinderprovider.h \
    zcameracalibratorwidget.h \
    zmulticameracalibratorwidget.h
SOURCES      += \
    zcalibrationimage.cpp \
    zcalibrationimagemodel.cpp \
    zcalibrationpatternfinder.cpp \
    zcameracalibratorworker.cpp \
    zcalibrationimageviewer.cpp \
    zmulticalibrationimage.cpp \
    zmulticalibrationimagemodel.cpp \
    zmulticameracalibratorworker.cpp \
    3rdParty/objectcontroller.cpp \
    zcalibrationpatternfinderprovider.cpp \
    zcameracalibratorwidget.cpp \
    zmulticameracalibratorwidget.cpp
FORMS += \
    zcameracalibratorwidget.ui \
    zmulticameracalibratorwidget.ui

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

###############################################################################
# Gui
include($$PWD/../../zgui/zgui.pri)

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
# Qt Solutions - Property Browser
include($$PWD/../../../3rdparty/qtpropertybrowser.pri)

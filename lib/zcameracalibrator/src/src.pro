include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#QT           -= gui // mientras use QPixmap necesito gui
QT           += widgets concurrent
TARGET        = $$qtLibraryTarget(zcameracalibrator)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_CAMERACALIBRATOR_LIBRARY

HEADERS      += \
    3rdParty/objectcontroller.h \
    zcalibrationimage.h \
    zcalibrationimagemodel.h \
    zcalibrationimageviewer.h \
    zcalibrationpatternfinder.h \
    zcalibrationpatternfinderplugininterface.h \
    zcalibrationpatternfinderprovider.h \
    zcameracalibrator_fwd.h \
    zcameracalibrator_global.h \
    zcameracalibratorwidget.h \
    zcameracalibratorworker.h \
    zmulticalibrationimage.h \
    zmulticalibrationimagemodel.h \
    zmulticameracalibratorwidget.h \
    zmulticameracalibratorworker.h \

SOURCES      += \
    3rdParty/objectcontroller.cpp \
    zcalibrationimage.cpp \
    zcalibrationimagemodel.cpp \
    zcalibrationimageviewer.cpp \
    zcalibrationpatternfinder.cpp \
    zcalibrationpatternfinderprovider.cpp \
    zcameracalibratorwidget.cpp \
    zcameracalibratorworker.cpp \
    zmulticalibrationimage.cpp \
    zmulticalibrationimagemodel.cpp \
    zmulticameracalibratorwidget.cpp \
    zmulticameracalibratorworker.cpp \

FORMS += \
    zcameracalibratorwidget.ui \
    zmulticameracalibratorwidget.ui \



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

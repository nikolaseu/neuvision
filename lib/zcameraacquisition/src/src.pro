include(../../../NEUVision.pri)

TEMPLATE      = lib
#QT           -= gui //we need gui because we use QImage, QScrollArea, etc
QT           += widgets
TARGET        = $$qtLibraryTarget(zcameraacquisition)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CAMERAACQUISITION_LIBRARY

HEADERS      += \
    Z3DCameraAcquisition \
    zcameraacquisition_fwd.h \
    zcameraacquisition_global.h \
    zcameraframesrecorder.h \
    zcameraimage.h \
    zcamerainfo.h \
    zcamerainterface.h \
    zcamerainterface_p.h \
    zcameralistmodel.h \
    zcameraplugininterface.h \
    zcamerapreviewer.h \
    zcameraprovider.h \
    zcameraselectordialog.h \
    zcameraselectorwidget.h \
    zcamerasettingswidget.h \
    zimageviewer.h \

SOURCES      += \
    zcameraframesrecorder.cpp \
    zcameraimage.cpp \
    zcamerainfo.cpp \
    zcamerainterface_p.cpp \
    zcameralistmodel.cpp \
    zcameraplugininterface.cpp \
    zcamerapreviewer.cpp \
    zcameraprovider.cpp \
    zcameraselectordialog.cpp \
    zcameraselectorwidget.cpp \
    zcamerasettingswidget.cpp \
    zimageviewer.cpp \

FORMS        += \
    zcamerapreviewer.ui \
    zcameraselectordialog.ui \
    zcameraselectorwidget.ui \
    zcamerasettingswidget.ui \

RESOURCES += \
    resources.qrc \



###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../../../3rdparty/qtpropertybrowser.pri)

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

###############################################################################
# Gui
include($$PWD/../../zgui/zgui.pri)

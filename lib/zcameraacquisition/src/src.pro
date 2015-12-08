include(../../../NEUVision.pri)

TEMPLATE      = lib
# CONFIG       += plugin
#QT           -= gui //we need gui because we use QImage, QScrollArea, etc
QT           += opengl # QGLWidget for ZImageViewer
greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
}
#QT           += core concurrent
TARGET        = $$qtLibraryTarget(zcameraacquisition)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CAMERAACQUISITION_LIBRARY
HEADERS      += \
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
    zcameraselectorwidget.h \
    zcamerasettingswidget.h \
    zimageviewer.h \
    zimageviewwidget.h \
    zcameraselectordialog.h

SOURCES      += \
    zcameraframesrecorder.cpp \
    zcameraimage.cpp \
    zcamerainfo.cpp \
    zcamerainterface_p.cpp \
    zcameralistmodel.cpp \
    zcamerapreviewer.cpp \
    zcameraprovider.cpp \
    zcameraselectorwidget.cpp \
    zcamerasettingswidget.cpp \
    zimageviewer.cpp \
    zimageviewwidget.cpp \
    zcameraselectordialog.cpp

FORMS        += \
    zcamerapreviewer.ui \
    zcameraselectorwidget.ui \
    zcamerasettingswidget.ui \
    zcameraselectordialog.ui

###############################################################################
# OpenCV
include($$PWD/../../../3rdparty/opencv.pri)

###############################################################################
# Qt Solutions - Property Browser
include($$PWD/../../../3rdparty/qtpropertybrowser.pri)

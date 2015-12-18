include(../../../NEUVision.pri)

TEMPLATE      = lib
greaterThan(QT_MAJOR_VERSION, 4) {
    QT -= gui
    QT += widgets
}
TARGET        = $$qtLibraryTarget(zcore)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CORE_LIBRARY
HEADERS += \
    zcore_global.h \
    zapplication.h \
    zpluginloader.h \
    zcoreplugin.h
SOURCES += \
    zapplication.cpp \
    zpluginloader.cpp \
    zcoreplugin.cpp

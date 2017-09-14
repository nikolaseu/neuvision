include(../../../NEUVision.pri)

TEMPLATE      = lib
QT           -= gui
TARGET        = $$qtLibraryTarget(zcore)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CORE_LIBRARY
HEADERS += \
    zcore_global.h \
    zpluginloader.h \
    zcoreplugin.h \
    zloghandler_p.h
SOURCES += \
    zpluginloader.cpp \
    zcoreplugin.cpp \
    zloghandler_p.cpp

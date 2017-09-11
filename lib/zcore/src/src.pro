include(../../../NEUVision.pri)

TEMPLATE      = lib
QT           -= gui # I guess widget depends on gui, so why?
QT           += widgets
TARGET        = $$qtLibraryTarget(zcore)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CORE_LIBRARY
HEADERS += \
    zcore_global.h \
    zapplication.h \
    zpluginloader.h \
    zcoreplugin.h \
    zloghandler_p.h \
    zapplicationstyle.h
SOURCES += \
    zapplication.cpp \
    zpluginloader.cpp \
    zcoreplugin.cpp \
    zloghandler_p.cpp \
    zapplicationstyle.cpp

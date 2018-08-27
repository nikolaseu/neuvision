include(../../../NEUVision.pri)

TEMPLATE      = lib
QT           -= gui
TARGET        = $$qtLibraryTarget(zcore)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_CORE_LIBRARY

HEADERS += \
    zcore_fwd.h \
    zcore_global.h \
    zcoreplugin.h \
    zloghandler_p.h \
    zpluginloader.h \
    zsettingsitem.h \
    qqmlobjectlistmodel.h

SOURCES += \
    zcoreplugin.cpp \
    zloghandler_p.cpp \
    zpluginloader.cpp \
    zsettingsitem.cpp \
    qqmlobjectlistmodel.cpp

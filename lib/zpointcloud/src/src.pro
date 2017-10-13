include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl 3drender
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY

HEADERS      += \
    zpointcloud.h \
    zpointcloud_fwd.h \
    zpointcloud_global.h \
    zpointcloudgeometry.h \
    zpointcloudplugininterface.h \
    zpointcloudprovider.h \
    zpointcloudreader.h \
    zpointfield.h \

SOURCES      += \
    zpointcloud.cpp \
    zpointcloudgeometry.cpp \
    zpointcloudplugininterface.cpp \
    zpointcloudprovider.cpp \
    zpointcloudreader.cpp \
    zpointfield.cpp \

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

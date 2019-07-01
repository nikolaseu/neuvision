include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl qml quick opengl 3dinput 3dquick 3drender 3dquickrender
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY

HEADERS      += \
    zpointcloud.h \
    zpointcloud_fwd.h \
    zpointcloud_global.h \
    zpointcloudgeometry.h \
    zpointcloudlistitem.h \
    zpointcloudlistmodel.h \
    zpointcloudplugininterface.h \
    zpointcloudprovider.h \
    zpointcloudreader.h \
    zpointfield.h \

SOURCES      += \
    zpointcloud.cpp \
    zpointcloudgeometry.cpp \
    zpointcloudlistitem.cpp \
    zpointcloudlistmodel.cpp \
    zpointcloudplugininterface.cpp \
    zpointcloudprovider.cpp \
    zpointcloudreader.cpp \
    zpointfield.cpp \

RESOURCES    += \
    zpointcloud_resources.qrc

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

###############################################################################
# QQmlObjectListModel
include($$PWD/../../../3rdparty/qqmlobjectlistmodel/qqmlobjectlistmodel.pri)

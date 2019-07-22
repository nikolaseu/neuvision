include(../../../NEUVision.pri)

VERSION       = $$Z3D_VERSION
TEMPLATE      = lib
#CONFIG       += staticlib
QT           += opengl qml quick opengl 3dinput 3dquick 3drender 3dquickrender
TARGET        = $$qtLibraryTarget(zpointcloud)
DESTDIR       = $$Z3D_BUILD_DIR
DEFINES      += Z3D_ZPOINTCLOUD_LIBRARY

INCLUDEPATH *= ../include

HEADERS      += \
    ../include/ZPointCloud/zpointcloud.h \
    ../include/ZPointCloud/zpointcloud_fwd.h \
    ../include/ZPointCloud/zpointcloud_global.h \
    ../include/ZPointCloud/zpointcloudgeometry.h \
    ../include/ZPointCloud/zpointcloudlistitem.h \
    ../include/ZPointCloud/zpointcloudlistmodel.h \
    ../include/ZPointCloud/zpointcloudplugininterface.h \
    ../include/ZPointCloud/zpointcloudprovider.h \
    ../include/ZPointCloud/zpointcloudreader.h \
    ../include/ZPointCloud/zpointfield.h \

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

include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui // we use QVector3D from gui
QT           += 3drender_private
TARGET        = $$qtLibraryTarget(zqtgeometryloaders)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/pointcloud
VERSION       = $$Z3D_VERSION

# since we use some files copied from Qt itself, to avoid conflicts we set another namespace
#DEFINES += 'QT_BEGIN_NAMESPACE=\"using namespace Qt3DRender; namespace Z3D::ZQtGeometryLoadersPlugin {\"'
#DEFINES += 'QT_END_NAMESPACE=}'

message(DEFINES $$DEFINES)

HEADERS       = \
    basegeometryloader_p.h \
    objgeometryloader.h \
    plygeometryloader.h \
    stlgeometryloader.h \
    zqtgeometryloaders.h \
    zqtgeometryloadersplugin.h

SOURCES       = \
    basegeometryloader.cpp \
    objgeometryloader.cpp \
    plygeometryloader.cpp \
    stlgeometryloader.cpp \
    zqtgeometryloaders.cpp \
    zqtgeometryloadersplugin.cpp

DISTFILES += \
    zqtgeometryloaders.json \

###############################################################################
# Point Cloud
include(../../zpointcloud.pri)

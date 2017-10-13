include(../../../../NEUVision.pri)

TEMPLATE      = lib
CONFIG       += plugin
#QT           -= gui // we use QVector3D from gui
TARGET        = $$qtLibraryTarget(zpointcloudpcl)
DESTDIR       = $$Z3D_BUILD_DIR/plugins/pointcloud
VERSION       = $$Z3D_VERSION

HEADERS       = \
    zpointcloudpclwrapper.h \
    zpointcloudlibraryplugin.h \
    
SOURCES       = \
    zpointcloudpclwrapper.cpp \
    zpointcloudlibraryplugin.cpp \
    
DISTFILES += \
    pcl.json \

###############################################################################
# Point Cloud
include(../../zpointcloud.pri)

###############################################################################
# PCL
include($$PWD/../../../../3rdparty/pcl.pri)

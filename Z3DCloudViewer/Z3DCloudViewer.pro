include(../NEUVision.pri)

QT += core qml quick opengl 3dinput 3dquick 3drender 3dquickrender
DESTDIR = $$Z3D_BUILD_DIR
TARGET = Z3DCloudViewer
VERSION = $$Z3D_VERSION
TEMPLATE = app

SOURCES += \
    main.cpp \

RESOURCES += \
    qml.qrc

###############################################################################
# Core
include($$PWD/../lib/zcore/zcore.pri)

###############################################################################
# Gui
include($$PWD/../lib/zgui/zgui.pri)

###############################################################################
# Points clouds
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

include(../NEUVision.pri)

QT       += core qml quick opengl 3dinput 3dquick 3drender 3dquickrender
DESTDIR = $$Z3D_BUILD_DIR
TARGET = Z3DCloudViewer
VERSION = $$Z3D_VERSION
TEMPLATE = app

SOURCES += \
    main.cpp \

RESOURCES += \
    qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

###############################################################################
# Core
include($$PWD/../lib/zcore/zcore.pri)

###############################################################################
# Gui
include($$PWD/../lib/zgui/zgui.pri)

###############################################################################
# Points clouds
include($$PWD/../lib/zpointcloud/zpointcloud.pri)

###############################################################################
# Point Cloud Library
include($$PWD/../3rdparty/pcl.pri)

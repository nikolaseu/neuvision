include(../../../NEUVision.pri)

TEMPLATE      = lib
QT           += widgets
TARGET        = $$qtLibraryTarget(zgui)
DESTDIR       = $$Z3D_BUILD_DIR
VERSION       = $$Z3D_VERSION
DEFINES      += Z3D_GUI_LIBRARY
HEADERS += \
    zapplication.h \
    zapplicationstyle.h \
    zgui_global.h \
    zmainwindow.h \
    zwidget.h
SOURCES += \
    zapplication.cpp \
    zapplicationstyle.cpp \
    zmainwindow.cpp \
    zwidget.cpp

mac {
    # Only include / compile these files on OS X
    OBJECTIVE_SOURCES += \
        osxutils.mm
    HEADERS  += \
        osxutils.h

    # Additionally include Cocoa for OS X code
    LIBS += -framework Foundation -framework Cocoa
    INCLUDEPATH += /System/Library/Frameworks/Foundation.framework/Versions/C/Headers
}

###############################################################################
# Core
include($$PWD/../../zcore/zcore.pri)

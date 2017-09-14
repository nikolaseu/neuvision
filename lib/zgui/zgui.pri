win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzgui1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzguid1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzgui

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzgui
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzgui_debug

    # this is for hide osx titlebar
    LIBS += -framework Foundation -framework Cocoa
    INCLUDEPATH += /System/Library/Frameworks/Foundation.framework/Versions/C/Headers
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

###############################################################################
# Core
include($$PWD/../zcore/zcore.pri)

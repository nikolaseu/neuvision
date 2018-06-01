win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibratord$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

###############################################################################
# Gui
include($$PWD/../zgui/zgui.pri)

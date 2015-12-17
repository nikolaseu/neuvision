win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlightd1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

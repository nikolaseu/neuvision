win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlightd$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzstructuredlight_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

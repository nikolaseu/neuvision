win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulation1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationd1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulation

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulation
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulation_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

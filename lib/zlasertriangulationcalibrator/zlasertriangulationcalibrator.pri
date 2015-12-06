win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationcalibrator1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationcalibratord1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationcalibrator

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationcalibrator
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzlasertriangulationcalibrator_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

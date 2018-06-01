win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcore$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcored$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcore

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcore
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcore_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

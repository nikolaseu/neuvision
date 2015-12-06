win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzpointcloudd1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

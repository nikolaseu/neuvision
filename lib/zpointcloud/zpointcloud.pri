win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzpointcloudd$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzpointcloud_debug
}

INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD/src

win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamerad$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamerad1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisitiond$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

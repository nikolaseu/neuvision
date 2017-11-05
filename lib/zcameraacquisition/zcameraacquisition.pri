win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisitiond1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration$$Z3D_VERSION_MAJOR
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrationd$$Z3D_VERSION_MAJOR
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

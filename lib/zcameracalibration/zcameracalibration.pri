win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration1
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrationd1
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibration_debug
}

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

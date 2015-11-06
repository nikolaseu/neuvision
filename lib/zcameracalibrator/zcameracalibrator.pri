win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibratord1
else:unix: LIBS += -L$$Z3D_BUILD_DIR -lzcameracalibrator

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

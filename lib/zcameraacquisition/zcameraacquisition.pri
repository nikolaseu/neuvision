win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisitiond1
else:unix: LIBS += -L$$Z3D_BUILD_DIR -lzcameraacquisition

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamerad1
else:unix: LIBS += -L$$Z3D_BUILD_DIR -lzcalibratedcamera

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

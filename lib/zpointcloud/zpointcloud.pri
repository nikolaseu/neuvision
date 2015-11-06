win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../../bin/ -lzpointcloud1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../../bin/ -lzpointcloudd1
else:unix: LIBS += -L$$OUT_PWD/../../../bin/ -lzpointcloud

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

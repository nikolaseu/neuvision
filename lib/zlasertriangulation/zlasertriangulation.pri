win32:CONFIG(release, debug|release): LIBS += -L$$DESTDIR -lzlasertriangulation1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$DESTDIR -lzlasertriangulationd1
else:unix: LIBS += -L$$DESTDIR -lzlasertriangulation

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

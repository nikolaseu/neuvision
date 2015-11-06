win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../lib/zlasertriangulationcalibrator/src/release/ -lzlasertriangulationcalibrator1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../lib/zlasertriangulationcalibrator/src/debug/ -lzlasertriangulationcalibratord1
else:unix: LIBS += -L$$OUT_PWD/../lib/zlasertriangulationcalibrator/src/ -lzlasertriangulationcalibrator

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src

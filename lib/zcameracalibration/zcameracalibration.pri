win32:CONFIG(release, debug|release): LIBS += -L$$DESTDIR -lzcameracalibration1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$DESTDIR -lzcameracalibrationd1
else:unix: LIBS += -L$$DESTDIR -lzcameracalibration

INCLUDEPATH += $$PWD/src
DEPENDPATH += $$PWD/src



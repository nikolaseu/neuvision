###############################################################################
# Qwt
!unix:include ( $$PWD/qwt-6.1.0/qwt.prf )

win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwtd
else:unix:LIBS += -L$$Z3D_BUILD_DIR -lqwt

INCLUDEPATH += $$PWD/qwt-6.1.0/src
DEPENDPATH += $$PWD/qwt-6.1.0/src

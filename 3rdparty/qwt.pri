###############################################################################
# Qwt
QWT_ROOT = $$PWD/qwt-6.1.2

include ( $$QWT_ROOT/qwt.prf )

win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwt
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwtd
else:unix:LIBS += -L$$Z3D_BUILD_DIR -lqwt

INCLUDEPATH += $$QWT_ROOT/src
DEPENDPATH += $$QWT_ROOT/src

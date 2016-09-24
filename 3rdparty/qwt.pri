###############################################################################
# Qwt
QWT_ROOT = $$PWD/qwt

include ( $$QWT_ROOT/qwt.prf )

win32: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwt
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lqwtd
}

unix:!macx: LIBS += -L$$Z3D_BUILD_DIR -lqwt

macx: {
    CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lqwt
    CONFIG(debug, debug|release):   LIBS += -L$$Z3D_BUILD_DIR -lqwt_debug
}

INCLUDEPATH += $$QWT_ROOT/src
DEPENDPATH += $$QWT_ROOT/src

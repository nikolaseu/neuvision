TEMPLATE     = lib
CONFIG      += plugin
TARGET       = $$qtLibraryTarget(zpinholeplugin)
VERSION      = 1.14.6.10
DESTDIR      = ../../../../bin/plugins/cameracalibration #DESTDIR      = $$[Z3D_CAMERACALIBRATION_INSTALL_PLUGINS]
HEADERS     += \
    pinholecameracalibration.h \
    pinholecameracalibrationplugin.h
SOURCES     += \
    pinholecameracalibration.cpp \
    pinholecameracalibrationplugin.cpp
OTHER_FILES += \
    pinhole.json

###############################################################################
# OpenCV
include($$PWD/../../../../3rdparty/opencv.pri)

###############################################################################
# Camera calibration
win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../../src/release/ -lzcamcal1
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../../src/debug/ -lzcamcald1
else:unix: LIBS += -L$$OUT_PWD/../../src/ -lzcamcal

INCLUDEPATH += $$PWD/../../src
DEPENDPATH += $$PWD/../../src

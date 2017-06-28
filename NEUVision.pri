# app version
Z3D_VERSION = 1.17.06.28

# define Z3D_VERSION_STR so we could use in main.cpp
Z3D_VERSTR = '\\"$${Z3D_VERSION}\\"'  # place quotes around the version string
DEFINES += Z3D_VERSION_STR=\"$${Z3D_VERSTR}\" # create a Z3D_VERSION_STR macro containing the version string

greaterThan(QT_MAJOR_VERSION, 4) {
    CONFIG(release, debug|release): Z3D_BUILD_DIR = $$PWD/bin/release5
    CONFIG(debug,   debug|release): Z3D_BUILD_DIR = $$PWD/bin/debug5
} else {
    CONFIG(release, debug|release): Z3D_BUILD_DIR = $$PWD/bin/release
    CONFIG(debug,   debug|release): Z3D_BUILD_DIR = $$PWD/bin/debug
}
#message($$Z3D_BUILD_DIR)

# enable C++14
CONFIG += c++14

# define Z3D_RELEASE when building in release mode
CONFIG(release, debug|release): DEFINES += Z3D_RELEASE

macx {
QMAKE_MAC_SDK = macosx10.12
}

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x050100    # disables all the APIs deprecated before Qt 5.8.0

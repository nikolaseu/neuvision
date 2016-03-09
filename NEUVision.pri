# app version
Z3D_VERSION = 1.15.12.25

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

# enable C++11
greaterThan(QT_MAJOR_VERSION, 4) {
    CONFIG += c++11
} else {
    QMAKE_CXXFLAGS += -std=c++0x
}

# define Z3D_RELEASE when building in release mode
CONFIG(release, debug|release): DEFINES += Z3D_RELEASE

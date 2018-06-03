# Need to discard STDERR so get path to NULL device
win32 {
    NULL_DEVICE = NUL # Windows doesn't have /dev/null but has NUL
} else {
    NULL_DEVICE = /dev/null
}

# Need to call git with manually specified paths to repository
# Include quotes for cases where the folder includes spaces!
BASE_GIT_COMMAND = git --git-dir \"$$PWD/.git\" --work-tree \"$$PWD\"

# Trying to get version from git tag / revision
GIT_VERSION = $$system($$BASE_GIT_COMMAND describe --always --tags --broken --long 2> $$NULL_DEVICE)
message(git describe --always --tags --broken --long: $$GIT_VERSION)

# Now we are ready to pass parsed version to Qt
Z3D_VERSION = $$GIT_VERSION
# Version can only be numerical so remove commit hash
Z3D_VERSION ~= s/\-(\d+)\-g[a-f0-9]{6,}.+/.\1/
# Create major minor patch & build
Z3D_VERSION_MAJOR = $$Z3D_VERSION
Z3D_VERSION_MAJOR ~= s/([0-9]+).[0-9]+.[0-9]+.[0-9]+/\1/
Z3D_VERSION_MINOR = $$Z3D_VERSION
Z3D_VERSION_MINOR ~= s/[0-9]+.([0-9]+).[0-9]+.[0-9]+/\1/
Z3D_VERSION_PATCH = $$Z3D_VERSION
Z3D_VERSION_PATCH ~= s/[0-9]+.[0-9]+.([0-9]+).[0-9]+/\1/
Z3D_VERSION_BUILD = $$Z3D_VERSION
Z3D_VERSION_BUILD ~= s/[0-9]+.[0-9]+.[0-9]+.([0-9]+)/\1/
message(App version $$Z3D_VERSION major $$Z3D_VERSION_MAJOR minor $$Z3D_VERSION_MINOR patch $$Z3D_VERSION_PATCH build $$Z3D_VERSION_BUILD)

# Adding C preprocessor #DEFINE so we can use it in C++ code
DEFINES += Z3D_VERSION_STR=\\\"$$Z3D_VERSION\\\"
DEFINES += Z3D_VERSION_BUILD_STR=\\\"$$GIT_VERSION\\\"

CONFIG(release, debug|release): Z3D_BUILD_DIR = $$PWD/bin/release5
CONFIG(debug,   debug|release): Z3D_BUILD_DIR = $$PWD/bin/debug5
message(Build folder should be $$Z3D_BUILD_DIR)

# enable C++14
CONFIG += c++14

# define Z3D_RELEASE when building in release mode
CONFIG(release, debug|release): DEFINES += Z3D_RELEASE

macx {
QMAKE_MAC_SDK = macosx10.13
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

message(Defines from NEUVision.pri $$DEFINES)

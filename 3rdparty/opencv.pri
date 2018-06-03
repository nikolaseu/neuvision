# Edit this section to make sure the paths match your system configuration

# Windows
win32 {
    OPENCV_DIR = C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    OPENCV_VER = 341

    OPENCV_INCLUDE_DIR = $$OPENCV_DIR/include
    CONFIG(release, debug|release) {
        OPENCV_LIB_DIR = $$OPENCV_DIR/lib
    } else {
        OPENCV_LIB_DIR = $$OPENCV_DIR/debug/lib
    }
}

# Linux
unix:!macx {
    OPENCV_INCLUDE_DIR = "/usr/local/include"
    OPENCV_LIB_DIR = "/usr/local/lib"
}

# macOS
macx: {
    OPENCV_INCLUDE_DIR = "/usr/local/opt/opencv/include"
    OPENCV_LIB_DIR = "/usr/local/opt/opencv/lib"
}

# Android
android: {
    include(opencv_android.pri)
}

CV_LIB_NAMES = core imgproc imgcodecs videoio calib3d features2d flann

for(lib, CV_LIB_NAMES) {
    CV_LIBS += -lopencv_$$lib
}

win32: {
    CONFIG(release, debug|release) {
        CV_LIB_PREFIX = $$OPENCV_VER
    } else {
        CV_LIB_PREFIX = $${OPENCV_VER}d
    }
    for(lib, CV_LIBS) {
        CV_LIBS_NEW += $$lib$$CV_LIB_PREFIX
    }
    CV_LIBS = $$CV_LIBS_NEW
}

LIBS += -L$$OPENCV_LIB_DIR $$CV_LIBS
INCLUDEPATH *= $$OPENCV_INCLUDE_DIR

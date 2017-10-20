# Edit this section to make sure the paths match your system configuration

# Windows
win32 {
    OPENCV_INCLUDE_DIR = $$(THIRDPARTY)/Release/include

    CONFIG(release, debug|release) {
        OPENCV_LIB_DIR = $$(THIRDPARTY)/Release/x64/vc14/lib
    } else {
        OPENCV_LIB_DIR = $$(THIRDPARTY)/Debug/x64/vc14/lib
    }

    OPENCV_VER = 330
}

# Linux
unix: {
    OPENCV_INCLUDE_DIR = "/usr/local/include"
    OPENCV_LIB_DIR = "/usr/local/lib"
    OPENCV_VER = 300
}

# Mac OSX
macx: {
    OPENCV_INCLUDE_DIR = "/usr/local/opt/opencv/include"
    OPENCV_LIB_DIR = "/usr/local/opt/opencv/lib"
    OPENCV_VER = 330
}

# Android
android: {
    include(opencv_android.pri)
}

lessThan(OPENCV_VER, 300) {
    # OpenCV 2
    CV_LIB_NAMES = core imgproc calib3d features2d flann contrib
} else {
    # OpenCV >= 3
    CV_LIB_NAMES = core imgproc imgcodecs videoio calib3d features2d flann
}

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
    CV_LIBS = $$CV_LIBS_NEW $$CV_EXT_LIBS
}

unix:!macx {
    QMAKE_LFLAGS += -Wl,-rpath=$$OPENCV_DIR/lib
}

LIBS += -L$$OPENCV_LIB_DIR $$CV_LIBS
INCLUDEPATH += $$OPENCV_INCLUDE_DIR

# Edit this section to make sure the paths match your system configuration

# Windows
win32 {
    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        OPENCV_INCLUDE_DIR = "D:/Nicou/Software/OpenCV-3.0.0-beta/include"
        OPENCV_LIB_DIR = "D:/Nicou/Software/OpenCV-3.0.0-beta/lib"
        OPENCV_VER = 300

        message('Using OpenCV x86 version '$$OPENCV_VER)
    } else {
        ## Windows x64 (64bit) specific build here
        OPENCV_INCLUDE_DIR = "C:/Users/T93907/Documents/OpenCV/2.4.9/opencv/build/include"
        OPENCV_LIB_DIR = "C:/Users/T93907/Documents/OpenCV/2.4.9/opencv/build/x64/vc12/lib"
        OPENCV_VER = 249

        message('Using OpenCV x86_64')
    }
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

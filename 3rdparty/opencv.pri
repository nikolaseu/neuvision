# Edit this section to make sure the paths match your system configuration

# Windows
win32 {
    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        #OPENCV_INCLUDE_DIR = "C:/Users/T93907/Documents/OpenCV/include"
        #OPENCV_LIB_DIR = "C:/Users/T93907/Documents/OpenCV/lib"
        #CV_VER = 246

        #OPENCV_INCLUDE_DIR = "D:/Nicou/Software/OpenCV/include"
        #OPENCV_LIB_DIR = "D:/Nicou/Software/OpenCV/lib"
        #CV_VER = 249

        #OPENCV_INCLUDE_DIR = "C:/Users/T93907/Documents/OpenCV-3.0.0-beta/include"
        #OPENCV_LIB_DIR = "C:/Users/T93907/Documents/OpenCV-3.0.0-beta/lib"
        #CV_VER = 300

        OPENCV_INCLUDE_DIR = "D:/Nicou/Software/OpenCV-3.0.0-beta/include"
        OPENCV_LIB_DIR = "D:/Nicou/Software/OpenCV-3.0.0-beta/lib"
        CV_VER = 300

        message('Using OpenCV x86 version '$$CV_VER)
    } else {
        ## Windows x64 (64bit) specific build here
        OPENCV_INCLUDE_DIR = "C:/Users/T93907/Documents/OpenCV/2.4.9/opencv/build/include"
        OPENCV_LIB_DIR = "C:/Users/T93907/Documents/OpenCV/2.4.9/opencv/build/x64/vc12/lib"
        CV_VER = 249

        message('Using OpenCV x86_64')
    }
}

# Linux
unix: {
    OPENCV_INCLUDE_DIR = "/usr/local/include"
    OPENCV_LIB_DIR = "/usr/local/lib"
}

lessThan(CV_VER, 300) {
    # OpenCV 2
    CV_LIB_NAMES = core imgproc calib3d features2d flann contrib highgui
} else {
    # OpenCV >= 3
    CV_LIB_NAMES = world
}

for(lib, CV_LIB_NAMES) {
    CV_LIBS += -lopencv_$$lib
}

win32: {
    CONFIG(release, debug|release) {
        CV_LIB_PREFIX = $$CV_VER
    } else {
        CV_LIB_PREFIX = $${CV_VER}d
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

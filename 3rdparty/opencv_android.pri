# Edit this section to make sure the paths match your system configuration
OPENCV_SDK_DIR = "/Users/nikolaseu/dev/OpenCV-android-sdk/sdk"
OPENCV_INCLUDE_DIR = $$OPENCV_SDK_DIR/native/jni/include
OPENCV_LIB_DIR = $$OPENCV_SDK_DIR/native/libs/armeabi-v7a

# These are only for android
LIBS += \
    $$OPENCV_LIB_DIR/libopencv_java3.so

OPENCV_3RDPARTY_LIB_DIR = $$OPENCV_SDK_DIR/native/3rdparty/libs/armeabi-v7a
LIBS += \
    $$OPENCV_3RDPARTY_LIB_DIR/liblibtiff.a \
    $$OPENCV_3RDPARTY_LIB_DIR/liblibjpeg.a \
    $$OPENCV_3RDPARTY_LIB_DIR/liblibjasper.a \
    $$OPENCV_3RDPARTY_LIB_DIR/liblibpng.a \
    $$OPENCV_3RDPARTY_LIB_DIR/libtbb.a \
    $$OPENCV_3RDPARTY_LIB_DIR/libIlmImf.a

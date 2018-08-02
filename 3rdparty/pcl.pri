################################################################################
## PCL
################################################################################
macx:{
    PCL_DIR = /usr/local/opt/pcl # from homebrew
    PCL_VERSION = 1.8
    INCLUDEPATH *= $$PCL_DIR/include/pcl-$$PCL_VERSION
    PCL_LIB_SUFFIX =
}

win32:{
#    PCL_DIR=C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    PCL_DIR = $$(PCL_DIR)
    INCLUDEPATH *= $$PCL_DIR/include
    PCL_LIB_SUFFIX = _release
}

LIBS += \
    -L$$PCL_DIR/lib/ \
        -lpcl_common$$PCL_LIB_SUFFIX \
        -lpcl_octree$$PCL_LIB_SUFFIX \
        -lpcl_io$$PCL_LIB_SUFFIX \
        -lpcl_kdtree$$PCL_LIB_SUFFIX \
        -lpcl_search$$PCL_LIB_SUFFIX \
        -lpcl_sample_consensus$$PCL_LIB_SUFFIX \
        -lpcl_filters$$PCL_LIB_SUFFIX \
        -lpcl_segmentation$$PCL_LIB_SUFFIX \
        -lpcl_visualization$$PCL_LIB_SUFFIX \
        -lpcl_features$$PCL_LIB_SUFFIX \
        -lpcl_surface$$PCL_LIB_SUFFIX \
        -lpcl_registration$$PCL_LIB_SUFFIX \
        -lpcl_keypoints$$PCL_LIB_SUFFIX \
        -lpcl_tracking$$PCL_LIB_SUFFIX \

################################################################################
## Boost
################################################################################
macx:{
    BOOST_DIR = /usr/local/opt/boost
    INCLUDEPATH *= $$BOOST_DIR/include
    BOOST_LIB_SUFFIX = -mt
}

win32:{
    BOOST_DIR = C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    INCLUDEPATH *= $$BOOST_DIR/include
    BOOST_LIB_SUFFIX = -vc140-mt
}

LIBS += \
    -L$$BOOST_DIR/lib/ \
        -lboost_system$$BOOST_LIB_SUFFIX \
        -lboost_thread$$BOOST_LIB_SUFFIX \

################################################################################
## FLANN
################################################################################
macx:{
    FLANN_DIR = /usr/local/opt/flann
    INCLUDEPATH *= $$FLANN_DIR/include
}

win32:{
    FLANN_DIR = C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    INCLUDEPATH *= $$FLANN_DIR/include
}

LIBS += \
    -L$$FLANN_DIR/lib \
        -lflann_cpp \

################################################################################
## Eigen
################################################################################
macx:{
    EIGEN_DIR = /usr/local/opt/eigen
    INCLUDEPATH *= $$EIGEN_DIR/include/eigen3
}

win32:{
    EIGEN_DIR = C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    INCLUDEPATH *= $$EIGEN_DIR/include
}

################################################################################
## PCL, Boost, etc etc
LIBS += \
    # PCL
    -L/usr/local/opt/pcl/lib/ \
        -lpcl_common \
        -lpcl_octree \
        -lpcl_io \
        -lpcl_kdtree \
        -lpcl_search \
        -lpcl_sample_consensus \
        -lpcl_filters \
        -lpcl_segmentation \
        -lpcl_visualization \
        -lpcl_features \
        -lpcl_surface \
        -lpcl_registration \
        -lpcl_keypoints \
        -lpcl_tracking \
#        -lpcl_apps \
    # Boost
    -L/usr/local/opt/boost/lib/ \
        -lboost_system \
        -lboost_filesystem \
        -lboost_thread-mt \
#        -lboost_date_time \
#        -lboost_iostreams \

INCLUDEPATH += \
#    $$PWD/include \
    /usr/local/opt/pcl/include/pcl-1.8 \
    /usr/local/opt/boost/include \
    /usr/local/opt/flann/include \
    /usr/local/opt/eigen/include/eigen3

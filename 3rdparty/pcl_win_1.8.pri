################################################################################
## PCL, VTK, etc etc
CONFIG(debug, debug|release) {
    LIBS += \
    -L$$(THIRDPARTY)/Debug/lib \
        -lpcl_common_debug \
        -lpcl_octree_debug \
        -lpcl_io_debug \
        -lpcl_kdtree_debug \
        -lpcl_search_debug \
        -lpcl_sample_consensus_debug \
        -lpcl_filters_debug \
        -lpcl_segmentation_debug \
        -lpcl_visualization_debug \
        -lpcl_features_debug \
        -lpcl_surface_debug \
        -lpcl_registration_debug \
        -lpcl_keypoints_debug \
        -lpcl_tracking_debug \
#        -lpcl_apps_debug \
    -L$$(THIRDPARTY)/Debug/lib \
        -llibboost_thread-vc140-mt-gd-1_65_1 \
}

CONFIG(release, debug|release) {
    LIBS += \
    -L$$(THIRDPARTY)/Release/lib \
        -lpcl_common_release \
        -lpcl_octree_release \
        -lpcl_io_release \
        -lpcl_kdtree_release \
        -lpcl_search_release \
        -lpcl_sample_consensus_release \
        -lpcl_filters_release \
        -lpcl_segmentation_release \
        -lpcl_visualization_release \
        -lpcl_features_release \
        -lpcl_surface_release \
        -lpcl_registration_release \
        -lpcl_keypoints_release \
        -lpcl_tracking_release \
#        -lpcl_apps_release \
    -L$$(THIRDPARTY)/Release/lib \
        -llibboost_thread-vc140-mt-1_65_1 \
}

LIBS += \
    -L$$(THIRDPARTY)/Release/lib \
        -lflann_cpp_s \

INCLUDEPATH += \
    $$(THIRDPARTY)/Release/include/pcl-1.8 \
    $$(THIRDPARTY)/Release/include/boost-1_65_1/ \
    $$(THIRDPARTY)/Release/include/eigen3/ \
    $$(THIRDPARTY)/Release/include \

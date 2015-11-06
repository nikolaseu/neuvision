################################################################################
## PCL, VTK, etc etc
CONFIG(debug, debug|release) {
    LIBS += \
    -L"C:/Program Files (x86)/PCL 1.6.0/lib/" \
        -lpcl_common_debug \
        -lpcl_octree_debug \
        -lpcl_io_debug \
        -lpcl_kdtree_debug \
        -lpcl_search_debug \
        -lpcl_sample_consensus_debug \
        -lpcl_filters_debug \
        -lpcl_segmentation_debug \
#        -lpcl_visualization_debug \
        -lpcl_features_debug \
        -lpcl_surface_debug \
        -lpcl_registration_debug \
        -lpcl_keypoints_debug \
        -lpcl_tracking_debug \
#        -lpcl_apps_debug \
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/Boost/lib/" \
#        -llibboost_system-vc100-mt-gd-1_49 \
#        -llibboost_filesystem-vc100-mt-gd-1_49 \
        -llibboost_thread-vc100-mt-gd-1_49 \
#        -llibboost_date_time-vc100-mt-gd-1_49 \
#        -llibboost_iostreams-vc100-mt-gd-1_49

}

CONFIG(release, debug|release) {
    LIBS += \
    -L"C:/Program Files (x86)/PCL 1.6.0/lib/" \
        -lpcl_common_release \
        -lpcl_octree_release \
        -lpcl_io_release \
        -lpcl_kdtree_release \
        -lpcl_search_release \
        -lpcl_sample_consensus_release \
        -lpcl_filters_release \
        -lpcl_segmentation_release \
#        -lpcl_visualization_release \
        -lpcl_features_release \
        -lpcl_surface_release \
        -lpcl_registration_release \
        -lpcl_keypoints_release \
        -lpcl_tracking_release \
#        -lpcl_apps_release \
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/Boost/lib/" \
#        -llibboost_system-vc100-mt-1_49 \
#        -llibboost_filesystem-vc100-mt-1_49 \
        -llibboost_thread-vc100-mt-1_49 \
#        -llibboost_date_time-vc100-mt-1_49 \
#        -llibboost_iostreams-vc100-mt-1_49

}

LIBS += \
        -ladvapi32 \ # no se por que, pero se necesita
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/FLANN/lib/" \
        -lflann_cpp_s \
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/Qhull/lib/" \
        -lqhullstatic \
#    -L"C:/Program Files (x86)/OpenNI/Lib/" \
#        -lopenNI \

INCLUDEPATH += \
    "C:/Program Files (x86)/PCL 1.6.0/include/pcl-1.6" \
    "C:/Program Files (x86)/PCL 1.6.0/3rdParty/Boost/include" \
    "C:/Program Files (x86)/PCL 1.6.0/3rdParty/Eigen/include" \
    "C:/Program Files (x86)/PCL 1.6.0/3rdParty/FLANN/include" \
    "C:/Program Files (x86)/PCL 1.6.0/3rdParty/Qhull/include" \




!greaterThan(QT_MAJOR_VERSION, 4) {
    CONFIG(debug, debug|release) {
        LIBS += \
        -L"C:/Program Files (x86)/PCL 1.6.0/lib/" \
            -lpcl_visualization_debug
    }

    CONFIG(release, debug|release) {
        LIBS += \
        -L"C:/Program Files (x86)/PCL 1.6.0/lib/" \
            -lpcl_visualization_release
    }
}

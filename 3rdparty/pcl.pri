################################################################################
## PCL
unix:include(pcl_lin_1.7.pri)
win32:include(pcl_win_1.6.pri)

################################################################################
## VTK
#unix: include(vtk_lin_5.10.pri)
unix: include(vtk_lin_6.1.pri)
win32:!greaterThan(QT_MAJOR_VERSION, 4) {
    include(vtk_win_5.8.pri)
}

################################################################################
## PCL
unix:!macx:include(pcl_lin_1.7.pri)
macx:include(pcl_osx_1.7.pri)
win32:include(pcl_win_1.6.pri)

################################################################################
## VTK
unix:!macx:include(vtk_lin_6.1.pri)
macx:include(vtk_osx_6.3.pri)
win32:!greaterThan(QT_MAJOR_VERSION, 4) {
    include(vtk_win_5.8.pri)
}

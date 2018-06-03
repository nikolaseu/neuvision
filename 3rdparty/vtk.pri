################################################################################
## VTK 8.x
################################################################################
macx:{
    VTK_VERSION=8.1
    VTK_DIR=/usr/local/opt/vtk
    INCLUDEPATH *= $$VTK_DIR/include/vtk-$$VTK_VERSION
}

win32:{
    VTK_VERSION=8.1
    VTK_DIR=C:/Tools/vcpkg/installed/x64-windows # assuming vcpkg installed at C:/Tools/vcpkg/
    INCLUDEPATH *= $$VTK_DIR/include
}

LIBS += \
    -L$$VTK_DIR/lib/ \
        -lvtkCommonCore-$$VTK_VERSION \
        -lvtkCommonDataModel-$$VTK_VERSION \
        -lvtkCommonExecutionModel-$$VTK_VERSION \
        -lvtkCommonMath-$$VTK_VERSION \
        -lvtkFiltersGeometry-$$VTK_VERSION \
        -lvtkFiltersSources-$$VTK_VERSION \
        -lvtkInteractionStyle-$$VTK_VERSION \
        -lvtkRenderingAnnotation-$$VTK_VERSION \
        -lvtkRenderingContextOpenGL2-$$VTK_VERSION \
        -lvtkRenderingCore-$$VTK_VERSION \
        -lvtkRenderingLOD-$$VTK_VERSION \
        -lvtkRenderingOpenGL2-$$VTK_VERSION \

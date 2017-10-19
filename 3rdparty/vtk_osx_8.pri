################################################################################
## VTK 8.0 for macOS X
################################################################################
INCLUDEPATH += \
    /usr/local/opt/vtk/include/vtk-8.1/

LIBS += \
    -L/usr/local/opt/vtk/lib/ \
        -lvtkCommonCore-8.1 \
        -lvtkCommonExecutionModel-8.1 \
        -lvtkCommonDataModel-8.1 \
        -lvtkRenderingCore-8.1 \
        -lvtkCommonMath-8.1 \
        -lvtkGUISupportQt-8.1 \
        -lvtkRenderingLOD-8.1 \
        -lvtkRenderingExternal-8.1 \
        -lvtkRenderingContextOpenGL2-8.1 \
        -lvtkRenderingOpenGL2-8.1 \
        -lvtkRenderingQt-8.1 \
        -lvtkFiltersSources-8.1 \
        -lvtkFiltersGeometry-8.1 \
        -lvtkInteractionStyle-8.1 \

################################################################################
## VTK
################################################################################
LIBS += \
    -L$$(THIRDPARTY)/Release/lib \
        -lvtkCommonCore-8.0 \
        -lvtkCommonExecutionModel-8.0 \
        -lvtkCommonDataModel-8.0 \
        -lvtkCommonMath-8.0 \
        -lvtkGUISupportQt-8.0 \
        -lvtkIOImage-8.0 \
        -lvtkRenderingCore-8.0 \
        -lvtkRenderingLOD-8.0 \
        -lvtkRenderingAnnotation-8.0 \
        -lvtkRenderingContextOpenGL2-8.0 \
        -lvtkRenderingOpenGL2-8.0 \
        -lvtkRenderingQt-8.0 \
        -lvtkFiltersSources-8.0 \
        -lvtkFiltersGeometry-8.0 \
        -lvtkInteractionStyle-8.0 \

INCLUDEPATH += \
    $$(THIRDPARTY)/Release/include/vtk-8.0/ \

################################################################################
## VTK
################################################################################

LIBS += \
    -L/usr/lib/vtk-5.10 \
        -lvtkCommon \
        -lvtkFiltering \
        -lvtkRendering \
        -lvtkGraphics \
        -lQVTK \

INCLUDEPATH += \
    /usr/include/vtk-5.10 \



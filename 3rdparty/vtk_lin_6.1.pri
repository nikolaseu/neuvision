################################################################################
## VTK
################################################################################
INCLUDEPATH += \
    /usr/include/vtk

LIBS += \
    -L/usr/lib/vtk \
        -lvtkCommonCore \
        -lvtkCommonDataModel \
        -lvtkRenderingCore \
        -lvtkCommonMath \
        -lvtkGUISupportQt \
        #-lvtkRenderingLOD \
        #-lvtkCommonExecutionModel \
        #-lvtksys \
        #-lQVTKWidgetPlugin \
        #-lvtkChartsCore \
        #-lvtkViewsQt \
        #-lvtkInteractionWidgets \
        #-lvtkInfovisCore \
        #-lvtkRenderingCore \
        #-lvtkRenderingOpenGL \
        #-lvtkImagingCore \
        #-lvtkIO \
        #-lvtkFiltering \
        #-lvtklibxml2 \
        #-lvtkDICOMParser \
        #-lvtkpng \
        #-lvtktiff \
        #-lvtkzlib \
        #-lvtkjpeg \
        #-lvtkalglib \
        #-lvtkexpat \
        #-lvtkverdict \
        #-lvtkmetaio \
        #-lvtkNetCDF \
        #-lvtksqlite \
        #-lvtkexoIIc \
        #-lvtkftgl \
        #lvtkfreetype \
        #-lvtkFiltersSources \

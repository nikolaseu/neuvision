################################################################################
## VTK 7.0 for Mac OSX
################################################################################
INCLUDEPATH += \
    /usr/local/opt/vtk/include/vtk-7.0/

LIBS += \
    -L/usr/local/opt/vtk/lib/ \
        -lvtkCommonCore-7.0 \
        -lvtkCommonDataModel-7.0 \
        -lvtkRenderingCore-7.0 \
        -lvtkCommonMath-7.0 \
        -lvtkGUISupportQt-7.0 \
        -lvtkRenderingLOD-7.0 \
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

################################################################################
## VTK 7.1 for Mac OSX
################################################################################
INCLUDEPATH += \
    /usr/local/opt/vtk/include/vtk-7.1/

LIBS += \
    -L/usr/local/opt/vtk/lib/ \
        -lvtkCommonCore-7.1 \
        -lvtkCommonDataModel-7.1 \
        -lvtkRenderingCore-7.1 \
        -lvtkCommonMath-7.1 \
        -lvtkGUISupportQt-7.1 \
        -lvtkRenderingLOD-7.1 \
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

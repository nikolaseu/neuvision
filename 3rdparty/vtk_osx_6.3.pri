################################################################################
## VTK 6.3 for Mac OSX
################################################################################
INCLUDEPATH += \
    /usr/local/opt/vtk/include/vtk-6.3/

LIBS += \
    -L/usr/local/opt/vtk/lib/ \
        -lvtkCommonCore-6.3 \
        -lvtkCommonDataModel-6.3 \
        -lvtkRenderingCore-6.3 \
        -lvtkCommonMath-6.3 \
        -lvtkGUISupportQt-6.3 \
        -lvtkRenderingLOD-6.3 \
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

################################################################################
## VTK
################################################################################

CONFIG(debug, debug|release) {
    LIBS += \
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/VTK/lib/vtk-5.8/" \
        -lvtkCommon-gd \
        -lvtkRendering-gd \
        -lvtkHybrid-gd \
        -lvtkGraphics-gd \
        -lvtkverdict-gd \
        -lvtkImaging-gd \
        -lvtkIO-gd \
        -lvtkFiltering-gd \
        -lvtkDICOMParser-gd \
        -lvtkNetCDF_cxx-gd \
        -lvtkmetaio-gd \
        -lvtksys-gd \
        -lvtksqlite-gd \
        -lvtkpng-gd \
        -lvtktiff-gd \
        -lvtkzlib-gd \
        -lvtkjpeg-gd \
        -lvtkexpat-gd \
        -lvtkftgl-gd \
        -lvtkfreetype-gd \
        -lvtkexoIIc-gd \
        -lvtkNetCDF-gd \
        -lQVTK-gd \
        -lvtkWidgets-gd
}

CONFIG(release, debug|release) {
    LIBS += \
    -L"C:/Program Files (x86)/PCL 1.6.0/3rdParty/VTK/lib/vtk-5.8/" \
        -lvtkCommon \
        -lvtkRendering \
        -lvtkHybrid \
        -lvtkGraphics \
        -lvtkverdict \
        -lvtkImaging \
        -lvtkIO \
        -lvtkFiltering \
        -lvtkDICOMParser \
        -lvtkNetCDF_cxx \
        -lvtkmetaio \
        -lvtksys \
        -lvtksqlite \
        -lvtkpng \
        -lvtktiff \
        -lvtkzlib \
        -lvtkjpeg \
        -lvtkexpat \
        -lvtkftgl \
        -lvtkfreetype \
        -lvtkexoIIc \
        -lvtkNetCDF \
        -lQVTK \
        -lvtkWidgets
}

INCLUDEPATH += \
    "C:/Program Files (x86)/PCL 1.6.0/3rdParty/VTK/include/vtk-5.8" \

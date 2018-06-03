TEMPLATE  = subdirs

SUBDIRS  += 3rdparty

SUBDIRS  += lib
lib.depends = 3rdparty

SUBDIRS  += Z3DCameraCalibration
Z3DCameraCalibration.depends = lib

SUBDIRS  += Z3DMultiCameraCalibration
Z3DMultiCameraCalibration.depends = lib

SUBDIRS  += Z3DCameraViewer
Z3DCameraViewer.depends = lib

SUBDIRS  += Z3DScanner
Z3DScanner.depends = lib

!win32:{
SUBDIRS  += Z3DCloudViewer
Z3DCloudViewer.depends = lib
}

message('Projects that will be built:' $$SUBDIRS)

OTHER_FILES += \
    README.md \
    Brewfile \
    .travis.yml \
    appveyor.yml \

TEMPLATE  = subdirs

SUBDIRS  += 3rdparty

SUBDIRS  += lib
lib.subdir  = Z3DMultiCameraCalibration/lib
lib.depends = 3rdparty

SUBDIRS  += src
src.subdir  = Z3DMultiCameraCalibration
src.depends = lib

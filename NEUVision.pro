TEMPLATE  = subdirs

SUBDIRS  += 3rdparty

SUBDIRS  += lib
lib.depends = 3rdparty

SUBDIRS  += Z3DCameraCalibration
Z3DCameraCalibration.depends = lib

SUBDIRS  += Z3DCameraViewer
Z3DCameraViewer.depends = lib

SUBDIRS  += LTSAcquisition
LTSAcquisition.depends = lib

SUBDIRS  += Z3DCloudViewer
Z3DCloudViewer.depends = lib

SUBDIRS  += Z3DScanner
Z3DScanner.depends = lib

greaterThan(QT_MAJOR_VERSION, 4) {
    # Qt >= 5
#    SUBDIRS  += CamAcquisition
#    CamAcquisition.depends = lib
} else {
    # Qt 4

#    SUBDIRS  += LTSAcquisition
#    LTSAcquisition.depends = lib





}

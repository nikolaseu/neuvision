TEMPLATE  = subdirs

SUBDIRS  += zcore \
    zstructuredlight

SUBDIRS  += zcalibratedcamera
zcalibratedcamera.depends = zcameraacquisition zcameracalibration

SUBDIRS  += zcameraacquisition

SUBDIRS  += zcameracalibration

SUBDIRS  += zcameracalibrator
zcameracalibrator.depends = zcalibratedcamera zcameraacquisition zcameracalibration

SUBDIRS  += zpointcloud
zpointcloud.depends = zcalibratedcamera zcameraacquisition zcameracalibration

SUBDIRS  += zlasertriangulation
zlasertriangulation.depends = zpointcloud

!unix:greaterThan(QT_MAJOR_VERSION, 4) {
    # Qt >= 5
} else {
    # Qt 4
#    SUBDIRS  += zlasertriangulation
#    zlasertriangulation.depends = zpointcloud

    SUBDIRS  += zlasertriangulationcalibrator
    zlasertriangulationcalibrator.depends = zcameracalibrator zpointcloud
}

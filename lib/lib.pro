TEMPLATE  = subdirs

SUBDIRS  += zcore

SUBDIRS  += zcameraacquisition
zcameraacquisition.depends = zcore

SUBDIRS  += zcameracalibration
zcameracalibration.depends = zcore

SUBDIRS  += zcalibratedcamera
zcalibratedcamera.depends = zcameraacquisition zcameracalibration

SUBDIRS  += zcameracalibrator
zcameracalibrator.depends = zcalibratedcamera zcameraacquisition zcameracalibration

!android:{
    SUBDIRS  += zpointcloud
    zpointcloud.depends = zcalibratedcamera zcameraacquisition zcameracalibration

    SUBDIRS  += zlasertriangulation
    zlasertriangulation.depends = zcalibratedcamera zcameraacquisition zcameracalibration zpointcloud
}

SUBDIRS  += zstructuredlight
zstructuredlight.depends = zcore zcalibratedcamera zcameraacquisition zcameracalibration zcameracalibrator

!unix:greaterThan(QT_MAJOR_VERSION, 4) {
    # Qt >= 5
} else {
    # Qt 4
#    SUBDIRS  += zlasertriangulation
#    zlasertriangulation.depends = zpointcloud
    !android:{
        SUBDIRS  += zlasertriangulationcalibrator
        zlasertriangulationcalibrator.depends = zcalibratedcamera zcameraacquisition zcameracalibration zcameracalibrator zpointcloud
    }
}

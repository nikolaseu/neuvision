TEMPLATE  = subdirs

SUBDIRS  += ../../lib/zcore

SUBDIRS  += ../../lib/zcameraacquisition
zcameraacquisition.depends = zcore

SUBDIRS  += ../../lib/zcameracalibration
zcameracalibration.depends = zcore

SUBDIRS  += ../../lib/zcalibratedcamera
zcalibratedcamera.depends = zcameraacquisition zcameracalibration

SUBDIRS  += ../../lib/zcameracalibrator
zcameracalibrator.depends = zcalibratedcamera zcameraacquisition zcameracalibration

SUBDIRS  += ../../lib/zstructuredlight
zstructuredlight.depends = zcore zcalibratedcamera zcameraacquisition zcameracalibration zcameracalibrator

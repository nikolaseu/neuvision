TEMPLATE  = subdirs

SUBDIRS  += zcore

SUBDIRS  += zgui
zgui.depends = zcore

SUBDIRS  += zcameraacquisition
zcameraacquisition.depends = zcore

SUBDIRS  += zcameracalibration
zcameracalibration.depends = zcore

SUBDIRS  += zcalibratedcamera
zcalibratedcamera.depends = zcameraacquisition zcameracalibration

SUBDIRS  += zcameracalibrator
zcameracalibrator.depends = zcalibratedcamera zcameraacquisition zcameracalibration

SUBDIRS  += zstructuredlight
zstructuredlight.depends = zcore zcalibratedcamera zcameraacquisition zcameracalibration zcameracalibrator

SUBDIRS  += zpointcloud
zpointcloud.depends = zcalibratedcamera zcameraacquisition zcameracalibration

message('Libraries that will be built:' $$SUBDIRS)

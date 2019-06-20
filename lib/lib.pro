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
zstructuredlight.depends = zcore zcalibratedcamera zcameraacquisition zcameracalibration zcameracalibrator zpointcloud

SUBDIRS  += zpointcloud
zpointcloud.depends = zcore

message('Libraries that will be built:' $$SUBDIRS)

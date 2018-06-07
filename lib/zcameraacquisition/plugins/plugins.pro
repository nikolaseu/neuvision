TEMPLATE  = subdirs

SUBDIRS += zsimulatedcamera
SUBDIRS += zopencvvideocapture
SUBDIRS += zqtcamera

unix:!android: {
    SUBDIRS += zlibgphoto2
}

## Uncomment the ones you want to build
win32:SUBDIRS  += zpylon
#SUBDIRS  += zavtvimba
#SUBDIRS  += zflycapture2
#SUBDIRS  += zlucam
#SUBDIRS  += zniimaqdxgrab
#SUBDIRS  += zpleoraebus

message('Configured for building camera acquisition plugins:' $$SUBDIRS)

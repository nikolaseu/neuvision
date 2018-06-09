TEMPLATE  = subdirs

# these are always built
SUBDIRS += zsimulatedcamera
SUBDIRS += zopencvvideocapture
SUBDIRS += zqtcamera

# zlibgphoto2 is only built on Mac and Linux (except android!)
unix:!android: {
    SUBDIRS += zlibgphoto2
}

# zpylon is included if env variable PYLON_DEV_DIR is set (it's the pylon SDK 'Development' folder)
PYLON_DEV_DIR = $$(PYLON_DEV_DIR)
isEmpty(PYLON_DEV_DIR){
    message('zpylon: PYLON_DEV_DIR not set, skipping plugin')
} else {
    message('zpylon: Building plugin using PYLON_DEV_DIR='$$PYLON_DEV_DIR)
    SUBDIRS  += zpylon
}

# zavtvimba is included if env variable AVT_VIMBA_DIR is set (it's the AVT Vimba SDK folder)
AVT_VIMBA_DIR = $$(AVT_VIMBA_DIR)
isEmpty(AVT_VIMBA_DIR){
    message('zavtvimba: AVT_VIMBA_DIR not set, skipping plugin')
} else {
    message('zavtvimba: Building plugin using AVT_VIMBA_DIR='$$AVT_VIMBA_DIR)
    SUBDIRS  += zavtvimba
}

## Uncomment the ones you want to build
#SUBDIRS  += zflycapture2
#SUBDIRS  += zlucam
#SUBDIRS  += zniimaqdxgrab
#SUBDIRS  += zpleoraebus

message('Configured for building camera acquisition plugins:' $$SUBDIRS)

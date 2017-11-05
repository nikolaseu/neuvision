TEMPLATE  = subdirs

SUBDIRS += zsimulatedcamera
SUBDIRS += zopencvvideocapture
SUBDIRS += zqtcamera

unix:!android: {
    SUBDIRS += zlibgphoto2
}

win32: {
    ## Windows build here
    SUBDIRS  += zpylon

    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        SUBDIRS  += zavtvimba
        SUBDIRS  += zflycapture2
        SUBDIRS  += zlucam
        SUBDIRS  += zniimaqdxgrab
        SUBDIRS  += zpleoraebus
    } else {
        ## Windows x64 (64bit) specific build here
        message('Building only x86_64 compatible camera acquisition plugins')
    }
}

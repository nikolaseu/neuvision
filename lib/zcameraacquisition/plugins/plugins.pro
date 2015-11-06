TEMPLATE  = subdirs

SUBDIRS += zsimulatedcamera
SUBDIRS += zopencvvideocapture

win32: {
    ## Windows build here

    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        SUBDIRS  += zavtvimba
        SUBDIRS  += zflycapture2
        SUBDIRS  += zlucam
        #SUBDIRS  += zniimaqdxgrab
        SUBDIRS  += zpleoraebus
    } else {
        ## Windows x64 (64bit) specific build here
        message('Building only x86_64 compatible camera acquisition plugins')
    }
}

# QCamera only included in Qt>=5
greaterThan(QT_MAJOR_VERSION, 4) {
    SUBDIRS  += zqtcamera
}



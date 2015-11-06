win32: {
    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        PGR_FLYCAPTURE_INSTALL_FOLDER = "C:/Program Files (x86)/Point Grey Research/FlyCapture2"
        LIBS += -L$$PGR_FLYCAPTURE_INSTALL_FOLDER/lib -lFlyCapture2 -lFlyCapture2GUI
        INCLUDEPATH += $$PGR_FLYCAPTURE_INSTALL_FOLDER/include
        DEPENDPATH  += $$PGR_FLYCAPTURE_INSTALL_FOLDER/include
    } else {
        ## Windows x64 (64bit) specific build here
        message('FlyCapture2 not configured for x86_64')
    }
}

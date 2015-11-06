win32: {
    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        LIBS += -L"C:/Program Files (x86)/National Instruments/Vision/Lib/MSVC/" -lnivision
        INCLUDEPATH += "C:/Program Files (x86)/National Instruments/Vision/Include"
        DEPENDPATH += "C:/Program Files (x86)/National Instruments/Vision/Include"
    } else {
        ## Windows x64 (64bit) specific build here
        message('NIVision not configured for x86_64')
    }
}

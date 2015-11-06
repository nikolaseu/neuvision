win32: {
    !contains(QMAKE_TARGET.arch, x86_64) {
        ## Windows x86 (32bit) specific build here
        LIBS += -L$$PWD/ -llucamapi
        INCLUDEPATH += $$PWD/
        DEPENDPATH  += $$PWD/
    } else {
        ## Windows x64 (64bit) specific build here
        message('LuCam not configured for x86_64')
    }
}

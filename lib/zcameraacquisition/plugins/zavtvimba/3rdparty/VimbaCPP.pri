win32:AVTVIMBA_DIR = "C:/Program Files/Allied Vision Technologies/AVTVimba_1.3"
win32:LIBS        += -L$$AVTVIMBA_DIR/VimbaCPP/Lib/Win32 -lVimbaCPP
win32:INCLUDEPATH +=   $$AVTVIMBA_DIR
win32:DEPENDPATH  +=   $$AVTVIMBA_DIR

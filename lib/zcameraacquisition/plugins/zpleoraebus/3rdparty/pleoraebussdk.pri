EBUS_SDK_DIR = "C:/Program Files (x86)/Pleora Technologies Inc/eBUS SDK"

unix|win32: LIBS += -L$$EBUS_SDK_DIR/Libraries/ -lPvBase

INCLUDEPATH += $$EBUS_SDK_DIR/Includes
DEPENDPATH += $$EBUS_SDK_DIR/Includes

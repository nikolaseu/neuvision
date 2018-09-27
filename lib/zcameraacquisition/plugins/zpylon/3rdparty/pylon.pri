win32: {
LIBS += -L$$(PYLON_DEV_DIR)/lib/x64 \
    -lGCBase_MD_VC120_v3_0_Basler_pylon_v5_0 \
    -lGenApi_MD_VC120_v3_0_Basler_pylon_v5_0 \
    -lPylonBase_MD_VC120_v5_0 \
    -lPylonC_MD_VC120 \
    -lPylonGUI_MD_VC120_v5_0 \
    -lPylonUtility_MD_VC120_v5_0

INCLUDEPATH += $$(PYLON_DEV_DIR)/include
}

macx: {
LIBS += -F$$(PYLON_DEV_DIR)/Frameworks/ -framework pylon
# to generate the include path that allows you to write #include <pylon/***.h>
QMAKE_CXXFLAGS += -F$$(PYLON_DEV_DIR)/Frameworks/
INCLUDEPATH += $$(PYLON_DEV_DIR)/Frameworks/pylon.framework/Headers/GenICam
}

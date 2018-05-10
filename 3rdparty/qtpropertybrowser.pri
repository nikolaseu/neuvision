win32:CONFIG(release, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lQtSolutionsPropertyBrowser
else:win32:CONFIG(debug, debug|release): LIBS += -L$$Z3D_BUILD_DIR -lQtSolutionsPropertyBrowser
else:unix: LIBS += -L$$Z3D_BUILD_DIR -lQtSolutionsPropertyBrowser

INCLUDEPATH += $$PWD/qtpropertybrowser/src
DEPENDPATH += $$PWD/qtpropertybrowser/src

DEFINES += QT_QTPROPERTYBROWSER_IMPORT

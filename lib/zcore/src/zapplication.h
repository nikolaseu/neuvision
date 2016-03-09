#pragma once

#include "zcore_global.h"

#include <QApplication>

class QSplashScreen;

namespace Z3D {

class Z3D_CORE_SHARED_EXPORT ZApplication : public QApplication
{
public:
    ZApplication(int &argc, char **argv);

    void loadPlugins();

    QSplashScreen *showSplashScreen();
    void finishSplashScreen(QWidget *widget);

private:
    QSplashScreen *m_splash;
};

} // namespace Z3D

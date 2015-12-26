#ifndef Z3D_CORE___ZAPPLICATION_H
#define Z3D_CORE___ZAPPLICATION_H

#include "zcore_global.h"

#include <QApplication>

class QSplashScreen;

namespace Z3D {

class Z3D_CORE_SHARED_EXPORT ZApplication : public QApplication
{
public:
    enum ZApplicationStyle {
        LightStyle,
        DarkStyle
    };

    ZApplication(int &argc, char **argv, ZApplicationStyle style = LightStyle);

    void loadPlugins();

    QSplashScreen *showSplashScreen();
    void finishSplashScreen(QWidget *widget);

private:
    QSplashScreen *m_splash;
};

} // namespace Z3D

#endif // Z3D_CORE___ZAPPLICATION_H

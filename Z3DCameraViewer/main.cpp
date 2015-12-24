#include <QSplashScreen>
#include <QThreadPool>

#include "zapplication.h"
#include <Z3DCameraProvider>
#include "zcameraselectorwidget.h"


int main(int argc, char *argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    ///
    Z3D::ZApplication app(argc,argv);

    int result;

    {
        QPixmap pixmap(":/splash.png");
        QSplashScreen splash(pixmap);
        splash.show();

        splash.showMessage("Loading plugins...");
        app.processEvents();
        app.loadPlugins();

        splash.showMessage("Loading camera acquisition plugins...");
        app.processEvents();
        Z3D::ZCameraProvider::loadPlugins();

        splash.showMessage("Loading main window...");
        app.processEvents();

#ifdef Z3D_THREAD_COUNT_LIMIT
        QThreadPool::globalInstance()->setMaxThreadCount(Z3D_THREAD_COUNT_LIMIT);
#endif

        Z3D::ZCameraSelectorWidget window;
        window.show();

        splash.finish(&window);

        result = app.exec();
    }

    qDebug() << "unloading camera plugins...";
    Z3D::ZCameraProvider::unloadPlugins();

    return result;
}

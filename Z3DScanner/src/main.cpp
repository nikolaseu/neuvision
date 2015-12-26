#include "ui/mainwindow.h"

#include <QSplashScreen>
#include <QSurfaceFormat>

#include "zapplication.h"
#include <Z3DCameraProvider>
#include <Z3DCameraCalibrationProvider>
#include "zcalibrationpatternfinderprovider.h"
#include "zpatternprojectionprovider.h"
#include "zstructuredlightsystemprovider.h"

#include "zscannerinitialconfigwizard.h"


int main(int argc, char* argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    ///
    Z3D::ZApplication app(argc, argv, Z3D::ZApplication::DarkStyle);

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    if (QCoreApplication::arguments().contains(QStringLiteral("--multisample")))
        fmt.setSamples(4);
    //if (QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"))) {
        fmt.setVersion(3, 2);
        fmt.setProfile(QSurfaceFormat::CoreProfile);
    //}
    QSurfaceFormat::setDefaultFormat(fmt);

    int result;

    {
        QPixmap pixmap(":/splash");
        QSplashScreen splash(pixmap);
        splash.show();

        splash.showMessage("Loading plugins...");
        app.processEvents();
        app.loadPlugins();

        splash.showMessage("Loading camera acquisition plugins...");
        app.processEvents();
        Z3D::ZCameraProvider::loadPlugins();

        splash.showMessage("Loading camera calibration plugins...");
        app.processEvents();
        Z3D::ZCameraCalibrationProvider::loadPlugins();

        splash.showMessage("Loading calibration pattern finder plugins...");
        app.processEvents();
        Z3D::ZCalibrationPatternFinderProvider::loadPlugins();

        splash.showMessage("Loading structured light system plugins...");
        app.processEvents();
        Z3D::ZStructuredLightSystemProvider::loadPlugins();

        splash.showMessage("Loading pattern projection plugins...");
        app.processEvents();
        Z3D::ZPatternProjectionProvider::loadPlugins();

        splash.showMessage("Loading main window...");
        app.processEvents();

#ifdef Z3D_THREAD_COUNT_LIMIT
        QThreadPool::globalInstance()->setMaxThreadCount(Z3D_THREAD_COUNT_LIMIT);
#endif

        //ZScannerInitialConfigWizard window;
        MainWindow window;

        splash.finish(&window);

        window.show();

        result = app.exec();
    }

    qDebug() << "unloading calibration pattern finder plugins...";
    Z3D::ZCalibrationPatternFinderProvider::unloadPlugins();

    qDebug() << "unloading camera calibration plugins...";
    Z3D::ZCameraCalibrationProvider::unloadPlugins();

    qDebug() << "unloading camera plugins...";
    Z3D::ZCameraProvider::unloadPlugins();

    return result;
}

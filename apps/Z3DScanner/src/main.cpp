//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zscannerqml.h"

#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraCalibration/zcameracalibrationprovider.h"
#include "ZCameraCalibrator/zcalibrationpatternfinderprovider.h"
#include "ZCore/zsettingsitem.h"
#include "ZGui/zapplication.h"
#include "ZGui/zapplicationstyle.h"
#include "ZPointCloud/zpointcloudprovider.h"
#include "ZStructuredLight/zpatternprojectionprovider.h"
#include "ZStructuredLight/zstructuredlightsystemprovider.h"

#if defined(Q_OS_MACOS)
#include "ZGui/zosxutils.h" // just to hide title bar in OSX
#include <QWindow>
#endif

#include <QDebug>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QSplashScreen>
#include <QSurfaceFormat>



int main(int argc, char* argv[])
{
    QSurfaceFormat glFormat;
    glFormat.setVersion(3, 2);
    glFormat.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(glFormat);

    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL, true);
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    ///
    Z3D::ZApplication app(argc, argv);

    Z3D::ZApplicationStyle::applyStyle(Z3D::ZApplicationStyle::DarkStyle);

    qDebug() << "available styles:" << QQuickStyle::availableStyles().join(", ");
    QQuickStyle::setStyle("Universal");
//    QQuickStyle::setStyle("Imagine");

    int result;

    {
        QQmlApplicationEngine engine;

        QSplashScreen &splash = *app.showSplashScreen();

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

        splash.showMessage("Loading point cloud plugins...");
        app.processEvents();

        //! TODO: This is ugly, find a better way to do this
        Z3D_ZPOINTCLOUD_INIT();
        Z3D_ZPOINTCLOUD_INIT_QMLENGINE(engine);

        /// Load config
        QDir configDir = QDir::current();
#if defined(Q_OS_WIN)
//        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
//            pluginsDir.cdUp();
#elif defined(Q_OS_MAC)
        if (configDir.dirName() == "MacOS") {
            configDir.cdUp();
            configDir.cdUp();
            configDir.cdUp();
        }
#endif

        QString settingsFile = configDir.absoluteFilePath(QString("%1.ini").arg(QApplication::applicationName()));
        qDebug() << "trying to load config from:" << settingsFile;
        QSettings settings(settingsFile, QSettings::IniFormat);

        splash.showMessage(QString("Loading settings file %1...").arg(settingsFile));
        app.processEvents();

        Z3D::ZStructuredLightSystemPtr structuredLightSystem = Z3D::ZStructuredLightSystemProvider::get(&settings);

        splash.showMessage("Loading main window...");
        app.processEvents();

        qmlRegisterUncreatableType<Z3D::ZSettingsItem>("Z3D.ZSettingsItem", 1, 0, "ZSettingsItem", "ZSettingsItem cannot be created, must be obtained from an object with settings");
        qRegisterMetaType<Z3D::ZSettingsItemModel*>("Z3D::ZSettingsItemModel*");

        engine.rootContext()->setContextProperty("scanner", QVariant::fromValue(new ZScannerQML(structuredLightSystem)));
        engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

#if defined(Q_OS_MACOS)
        // just to hide title bar in OSX
        for (auto &obj : engine.rootObjects()) {
            if (auto win = qobject_cast<QWindow*>(obj)) {
                Z3D::ZOSXUtils::hideTitleBar(win->winId());
                Z3D::ZOSXUtils::applyTranslucentBackgroundEffect(win->winId());
                break;
            }
        }
#endif

        splash.hide();

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

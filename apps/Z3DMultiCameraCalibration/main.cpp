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

#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraCalibration/zcameracalibrationprovider.h"
#include "ZCameraCalibrator/zcalibrationpatternfinderprovider.h"
#include "ZCameraCalibrator/zmulticameracalibratorwidget.h"
#include "ZCore/zlogging.h"
#include "ZGui/zapplication.h"
#include "ZGui/zapplicationstyle.h"

#include <QDir>
#include <QSettings>
#include <QSplashScreen>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.apps.zmulticameracalibration", QtInfoMsg)

int main(int argc, char* argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    ///
    Z3D::ZApplication app(argc, argv);

    Z3D::ZApplicationStyle::applyStyle(Z3D::ZApplicationStyle::DarkStyle);

    int result;

    {
        QSplashScreen &splash = *app.showSplashScreen();

        splash.showMessage("Loading plugins...");
        Z3D::ZApplication::processEvents();
        app.loadPlugins();

        splash.showMessage("Loading camera acquisition plugins...");
        Z3D::ZApplication::processEvents();
        Z3D::ZCameraProvider::loadPlugins();

        splash.showMessage("Loading camera calibration plugins...");
        Z3D::ZApplication::processEvents();
        Z3D::ZCameraCalibrationProvider::loadPlugins();

        splash.showMessage("Loading calibration pattern finder plugins...");
        Z3D::ZApplication::processEvents();
        Z3D::ZCalibrationPatternFinderProvider::loadPlugins();

        splash.showMessage("Loading main window...");
        Z3D::ZApplication::processEvents();

        /// TODO this is just a workaround for now, see how to improve it
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
        zDebug() << "trying to load config from:" << settingsFile;
        QSettings settings(settingsFile, QSettings::IniFormat);

        Z3D::ZCameraList cameras;
        settings.beginGroup("Left");
        {
            cameras.push_back(Z3D::ZCameraProvider::getCamera(&settings));
        }
        settings.endGroup();

        settings.beginGroup("Right");
        {
            cameras.push_back(Z3D::ZCameraProvider::getCamera(&settings));
        }
        settings.endGroup();

        Z3D::ZMultiCameraCalibratorWidget calibWidget(cameras);
        calibWidget.show();

        splash.finish(&calibWidget);

        result = app.exec();
    }

    zDebug() << "unloading calibration pattern finder plugins...";
    Z3D::ZCalibrationPatternFinderProvider::unloadPlugins();

    zDebug() << "unloading camera calibration plugins...";
    Z3D::ZCameraCalibrationProvider::unloadPlugins();

    zDebug() << "unloading camera plugins...";
    Z3D::ZCameraProvider::unloadPlugins();

    return result;
}

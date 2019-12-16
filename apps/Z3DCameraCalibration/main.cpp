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

#include "mainwindow.h"

#include <QSplashScreen>

#include "ZCameraAcquisition/zcameraprovider.h"
#include "ZCameraCalibration/zcameracalibrationprovider.h"
#include "ZCameraCalibrator/zcalibrationpatternfinderprovider.h"
#include "ZGui/zapplication.h"
#include "ZGui/zapplicationstyle.h"

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

        splash.showMessage("Loading main window...");
        app.processEvents();

        MainWindow window;
        window.show();

        splash.finish(&window);

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

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
#include "ZCameraAcquisition/zcameraselectorwidget.h"
#include "ZCore/zlogging.h"
#include "ZGui/zapplication.h"
#include "ZGui/zapplicationstyle.h"

#include <QSplashScreen>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.apps.zcameraviewer", QtInfoMsg)

int main(int argc, char *argv[])
{
    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    ///
    Z3D::ZApplication app(argc,argv);

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

        splash.showMessage("Loading main window...");
        Z3D::ZApplication::processEvents();

        Z3D::ZCameraSelectorWidget window;
        window.show();

        splash.finish(&window);

        result = app.exec();
    }

    zDebug() << "unloading camera plugins...";
    Z3D::ZCameraProvider::unloadPlugins();

    return result;
}

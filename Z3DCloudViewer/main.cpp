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

#include "zapplication.h"
#include "ZPointCloud/zpointcloud.h"
#include "ZPointCloud/zpointcloudgeometry.h"
#include "ZPointCloud/zpointcloudprovider.h"
#include "zpointcloudviewercontroller.h"

#if defined(Q_OS_MACOS)
#include "zosxutils.h" // just to hide title bar in OSX
#include <QWindow>
#endif

#include <QQmlApplicationEngine>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QSurfaceFormat glFormat;
    glFormat.setVersion(3, 3);
    glFormat.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(glFormat);

    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL, true);

    Z3D::ZApplication app(argc, argv);
    app.loadPlugins();
    Z3D::ZPointCloudProvider::loadPlugins();

    QQmlApplicationEngine engine;

    //! TODO: This is ugly, find a better way to do this
    Z3D_ZPOINTCLOUD_INIT();
    Z3D_ZPOINTCLOUD_INIT_QMLENGINE(engine);

    qmlRegisterType<Z3D::ZPointCloudViewerController>("Z3D.ZPointCloudViewer", 1, 0, "ZPointCloudViewerController");

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

    return app.exec();
}

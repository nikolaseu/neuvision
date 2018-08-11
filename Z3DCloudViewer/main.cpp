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
#include "zpointcloud.h"
#include "zpointcloudgeometry.h"
#include "zpointcloudprovider.h"
#include "zpointcloudreader.h"

#include <QQmlApplicationEngine>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{
    QSurfaceFormat glFormat;
    glFormat.setVersion(3, 2);
    glFormat.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(glFormat);

    /// to print out diagnostic information about each plugin it (Qt) tries to load
    //qputenv("QT_DEBUG_PLUGINS", "1");

    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL, true);

    Z3D::ZApplication app(argc, argv);
    app.loadPlugins();
    Z3D::ZPointCloudProvider::loadPlugins();

    qmlRegisterUncreatableType<Z3D::ZPointCloud>("Z3D.PointCloud", 1, 0, "PointCloud", "ZPointCloud cannot be created, must be obtained from a PointCloudReader");
    qmlRegisterType<Z3D::ZPointCloudReader>("Z3D.PointCloud", 1, 0, "PointCloudReader");
    qmlRegisterType<Z3D::ZPointCloudGeometry>("Z3D.PointCloud", 1, 0, "PointCloudGeometry");

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));

    return app.exec();
}
